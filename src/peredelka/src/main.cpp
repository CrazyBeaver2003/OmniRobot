#include <chrono>
#include <memory>
#include <string>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/float32.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using namespace std::chrono_literals;

class RobotController : public rclcpp::Node
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  RobotController() : Node("robot_controller")
  {
    // Подписка на данные лидара
    scan_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", 10, std::bind(&RobotController::scan_callback, this, std::placeholders::_1));
    
    // Публикация команд скорости
    cmd_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    
    // Клиент для отправки целей навигации
    goal_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

    // Инициализация трансформ-лиссенера
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Таймер для выполнения цикла управления
    timer_ = this->create_wall_timer(100ms, std::bind(&RobotController::control_loop, this));
  }

private:
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    min_distance_ = std::numeric_limits<float>::infinity();
    for (const auto &range : msg->ranges) {
      if (range < min_distance_) {
        min_distance_ = range;
      }
    }
  }

  void control_loop()
  {
    if (min_distance_ < safe_distance_) {
      auto cmd_msg = geometry_msgs::msg::Twist();
      cmd_msg.angular.z = 0.5;  // Поворот для избегания препятствия
      cmd_publisher_->publish(cmd_msg);
    } else {
      auto goal_msg = NavigateToPose::Goal();
      goal_msg.pose.header.frame_id = "map";
      goal_msg.pose.header.stamp = this->get_clock()->now();
      goal_msg.pose.pose.position.x = target_x_;
      goal_msg.pose.pose.position.y = target_y_;
      goal_msg.pose.pose.orientation.w = 1.0;

      auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
      send_goal_options.result_callback = std::bind(&RobotController::goal_response_callback, this, std::placeholders::_1);
      goal_client_->async_send_goal(goal_msg, send_goal_options);
    }
  }

  void goal_response_callback(std::shared_future<GoalHandleNavigateToPose::SharedPtr> future)
  {
    auto goal_handle = future.get();
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");

    auto result_future = goal_client_->async_get_result(goal_handle);
    result_future.wait();

    if (result_future.get().code != rclcpp_action::ResultCode::SUCCEEDED) {
      RCLCPP_ERROR(this->get_logger(), "Goal failed");
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Goal succeeded!");
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_publisher_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr goal_client_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  float min_distance_;
  const float safe_distance_ = 0.5;  // Безопасное расстояние от препятствий
  float target_x_ = 2.0;  // Пример целевой позиции
  float target_y_ = 2.0;  // Пример целевой позиции
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RobotController>());
  rclcpp::shutdown();
  return 0;
}
