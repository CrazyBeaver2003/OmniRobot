#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include <termios.h>
#include <stdio.h>

static struct termios oldt, newt;

int getch()
{
  static bool firstTime = true;

  if (firstTime)
    {
      firstTime = false;
      tcgetattr( STDIN_FILENO, &oldt);
      newt = oldt;
      newt.c_lflag &= ~(ICANON);
      tcsetattr( STDIN_FILENO, TCSANOW, &newt);
    }

  int c = getchar();

  return c;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("keyboard_node");
  auto publisher = node->create_publisher<std_msgs::msg::Int32MultiArray>("keyboard_topic", 1);

  std_msgs::msg::Int32MultiArray message;

  rclcpp::WallRate loop_rate(100);
  while (rclcpp::ok())
  {
    int c = getch();
    switch(c)
    {
    case 32: // space
      message.data = {1, 1};
      break;
    case 10: // enter
      message.data = {2, 2};
      break;   
    default:
      message.data = {0, 0};
      break;
    }

    RCLCPP_INFO(node->get_logger(), "Publishing: '%d'", c);
    publisher->publish(message);

    rclcpp::spin_some(node);
    loop_rate.sleep();
  }

  tcsetattr( STDIN_FILENO, TCSANOW, &oldt);

  rclcpp::shutdown();

  return 0;
}