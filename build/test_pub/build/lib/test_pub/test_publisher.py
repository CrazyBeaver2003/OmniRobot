import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32


class TestPub(Node):

    def __init__(self):
        super().__init__('test_topic_publisher')
        self.test_publisher_ = self.create_publisher(
            Int32,
            'distance_color_topic', ############################################################## Print Here Topic To Test ##############################################################
            10)
        self.counter = 0
        timer_period = 0.1  # seconds
        self.timer_pub = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        self.counter += 1
        msg = Int32()
        if(self.counter % 2 == 0):
            msg.data = 1
        else:
            msg.data = 0
        self.test_publisher_.publish(msg)
        self.get_logger().info('TEST_NODE: Publish "%d"' % msg.data)

def main(args=None):
    rclpy.init(args=args)

    test_node = TestPub()

    rclpy.spin(test_node)

    test_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
