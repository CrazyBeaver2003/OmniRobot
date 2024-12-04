import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray
import math

class LidarDistanceNode(Node):
    def __init__(self):
        super().__init__('lidar_template_node')
        self.publisher_distances = self.create_publisher(Float32MultiArray, 'lidar_distance', 10)
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',  # Replace with the actual lidar topic name
            self.lidar_callback,
            10)
        self.subscription  # prevent unused variable warning
        threshold_angle = 0.0

    def lidar_callback(self, msg):
        angles = [msg.angle_min + i * msg.angle_increment for i in range(len(msg.ranges))]
        distances = msg.ranges
        min_distance_index = next((i for i, d in enumerate(distances) if d < 0.2), None)
                
        if min_distance_index is not None:       
            self.threshold_angle = angles[min_distance_index] 
            self.publisher_distances.publish(Float32MultiArray(data = [self.threshold_angle]))

        # Print the distances
        self.get_logger().info('Front distance: %f' % self.threshold_angle)


def main(args=None):
    rclpy.init(args=args)
    lidar_distance_node = LidarDistanceNode()
    rclpy.spin(lidar_distance_node)
    lidar_distance_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()