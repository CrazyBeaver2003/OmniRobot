import eventlet
import serial
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from time import sleep
from std_msgs.msg import String
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Int32
from std_msgs.msg import Float32

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('motor_topic_subscriber')
        self.distance_publisher = self.create_publisher(Float32, 'ultra_sound_distance_topic', 10)
        self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=10)
        self.subscription = self.create_subscription(
             Float32MultiArray,
            'motor_control_topic',
            self.listener_callback,
            10)
        self.ser.flush()
        self.str_mess = ""
        timer_period = 0.1   # seconds
        self.algorithm_cycle = self.create_timer(timer_period, self.measurement)

    def listener_callback(self, msg):
        self.ser.write(str(msg.data[0]).encode())
        self.ser.write(b",")
        self.ser.write(str(msg.data[1]).encode())
        self.ser.write(b",")
        self.ser.write(str(msg.data[2]).encode())
        self.ser.write(b",")
        self.ser.write(str(msg.data[3]).encode())
        self.ser.write(b"\n")
        self.ser.reset_input_buffer()

    def measurement(self):
        msg = Float32()
        if self.ser.in_waiting > 0:
            self.str_mess = self.ser.readline().decode('utf-8').rstrip()
            try:
                dist = float(self.str_mess)
                if (dist >0.0 and dist < 300.0):
                    msg.data = dist
                else:
                    msg.data = -1.0
            except:
                self.ser.flush()
                msg.data = -1.0
                return
        self.distance_publisher.publish(msg)   
        self.get_logger().info(str(msg.data).encode())
        
def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()
    #minimal_subscriber.measurement()
    rclpy.spin(minimal_subscriber)

    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
