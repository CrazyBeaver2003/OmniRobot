#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "serial/serial.h"

class ArduinoCommunicator : public rclcpp::Node
{
public:
  ArduinoCommunicator()
  : Node("arduino_communicator")
  {
    this->declare_parameter<std::string>("serial_port", "/dev/ttyACM0");
    this->declare_parameter<int>("baud_rate", 115200);

    serial_port_ = this->get_parameter("serial_port").as_string();
    baud_rate_ = this->get_parameter("baud_rate").as_int();

    ser_.setPort(serial_port_);
    ser_.setBaudrate(baud_rate_);
    serial::Timeout to = serial::Timeout::simpleTimeout(1000);
    ser_.setTimeout(to);
    ser_.open();

    if(ser_.isOpen()) {
      RCLCPP_INFO(this->get_logger(), "Serial port initialized");
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to open serial port");
      rclcpp::shutdown();
    }

    subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10, std::bind(&ArduinoCommunicator::twist_callback, this, std::placeholders::_1));
  }

private:
  void twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    if(ser_.isOpen()) {
      std::stringstream ss;
      ss << msg->linear.x << "," << msg->linear.y << "," << msg->angular.z << "," << "\n";
      ser_.write(ss.str());
      RCLCPP_INFO(this->get_logger(), "Sent: '%s'", ss.str().c_str());
    } else {
      RCLCPP_ERROR(this->get_logger(), "Serial port not open");
    }
  }

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
  serial::Serial ser_;
  std::string serial_port_;
  int baud_rate_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ArduinoCommunicator>());
  rclcpp::shutdown();
  return 0;
}