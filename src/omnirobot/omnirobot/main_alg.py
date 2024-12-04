import asyncio
import eventlet
import rclpy
from rclpy.node import Node
from time import sleep
from std_msgs.msg import Int32
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import String
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
import math
from sensor_msgs.msg import LaserScan

class MainAlg(Node):
    __flag = 1 
    __AruCoFlag = 0
    __TipTap = 0
    __Blue = 0
    pointer_dist_x= 0
    pointer_dist_y= 0
    distance1 = 0
    distance2 = 0
    distance3 = 0
    AruCo = [-1,-1]
    audio = -1
    AruCo_Color = "None"
    AruCoColorFlag = "None"
    color = "None"
    speedx = 0.0
    speedy = 0.0
    angvel = 0.0
    servo = 0.0
    sound_played = 0
    threshold_angle = 0.0
    obstacle = 0
    centered = 0
    captured = 0
    us_distance = 0.0
    def __init__(self):

        super().__init__('MainAlg')
        #self.distance_color_pub = self.create_publisher(Int32, 'distance_color_topic', 10)
        self.motor_pub = self.create_publisher(Float32MultiArray, 'motor_control_topic', 10)
        #self.servo_pub = self.create_publisher(Int32, 'servo_control_topic', 10)
        self.audio_pub = self.create_publisher(Int32, 'robot_audio_topic', 1)
        #self.distance_sub = self.create_subscription(Int32MultiArray, 'distance_topic', self.distance_callback, 10)
        #self.color_sub = self.create_subscription(String, 'color_topic', self.color_callback, 10)
        #self.aruco_sub = self.create_subscription(Int32MultiArray, 'aruco_topic', self.aruco_callback, 10)
        self.ultra_sound_sub = self.create_subscription(Float32, 'ultra_sound_distance_topic', self.us_callback, 10)
        self.pointer_sub = self.create_subscription(Int32MultiArray, 'barrel_topic', self.pointer_callback, 10)
        self.lidar_sub = self.create_subscription(LaserScan,
            'scan',  # Replace with the actual lidar topic name
            self.lidar_callback,
            10)
        timer_period = 0.1    # seconds
        self.algorithm_cycle = self.create_timer(timer_period, self.main_algorithm)

    def main_algorithm(self):
        if (self.obstacle == 0 and self.centered == 0 and self.captured == 0):
            if (abs(self.pointer_dist_x)<1000):
                self.speedx = -0.6 * ((self.pointer_dist_y)/240)
                self.speedy = 0.0
                self.angvel = -1.9 * (self.pointer_dist_x/300)
                if (abs(self.pointer_dist_y)<30 and abs(self.pointer_dist_x)<30):
                    self.audio_publisher(2) 
                    
                else: 
                    self.servo = 0.0
                    self.centered = 0
                    self.audio_publisher(1)
            else:
                self.speedx = 0.0
                self.speedy = 0.0
                self.angvel = 0.0
                self.audio_publisher(3) 
        elif (self.obstacle == 0 and self.centered == 99 and self.captured == 0):
            self.speedx = 0.1
            self.speedy = 0.0
            self.angvel = 0.0
            self.get_logger().info('Catching.."%f"' %self.us_distance)
            if (self.us_distance < 10.0 and self.us_distance >0):
                self.captured = 1
                self.speedx = 0.1
                self.speedy = 0.0
                self.angvel = 0.0
                self.motor_publisher() 
                eventlet.sleep(2)
                self.servo = 60.0
                self.speedx = 0.0
                self.centered = 0
                self.audio_publisher(5)
        elif (self.obstacle == 0 and self.captured ==99):
            self.speedx = 0.0
            self.speedy = 0.0
            self.angvel = 2.0
            self.motor_publisher() 
            eventlet.sleep(3.5)
            self.speedx = 0.0
            self.speedy = 0.0
            self.angvel = 0.0
            self.servo = 0.0
            self.motor_publisher() 
            eventlet.sleep(1)
            self.speedx = -0.2
            self.speedy = 0.0
            self.angvel = 0.0
            self.motor_publisher() 
            eventlet.sleep(2)
            self.speedx = 0.0
            self.speedy = 0.0
            self.angvel = -2.0
            self.motor_publisher() 
            eventlet.sleep(3.5)
            self.captured = 0
            
        else:
            self.get_logger().info('OBSTACLE DETECTED "%f"' %self.threshold_angle)
            self.audio_publisher(4) #gg
            linear_speed = -0.2
            angular_speed = 0.0  # Нет вращения

            # Преобразуем угол в радианы
            angle_rad = self.threshold_angle + math.pi

            # Используем тригонометрические функции для вычисления компонент вектора
            self.speedx = linear_speed * math.cos(angle_rad)
            self.speedy = linear_speed * math.sin(angle_rad)
            self.ang_vel = angular_speed
        self.motor_publisher()    

    def lidar_callback(self, msg):
        angles = [msg.angle_min + i * msg.angle_increment for i in range(len(msg.ranges))]
        distances = msg.ranges
        min_distance_index = next((i for i, d in enumerate(distances) if d < 0.2), None)
              
        if min_distance_index is not None:       
            self.threshold_angle = angles[min_distance_index]
            self.obstacle = 1
        else:
            self.obstacle = 0     



    def pointer_callback(self, msg):
        if(len(msg.data) >= 2):
            try:
                self.pointer_dist_x = msg.data[0]
            except:
                pass
            try:
                self.pointer_dist_y = msg.data[1]
            except:
                pass   
    def us_callback(self, msg):
        self.us_distance = msg.data
        self.get_logger().info('distance.."%f"' %msg.data)            
    '''
    def CD_publisher(self, servo_mess):
        msg = Int32()
        msg.data = servo_mess
        self.distance_color_pub.publish(msg)'''

    def motor_publisher(self):
        msg = Float32MultiArray()
        msg.data = [self.speedx, self.speedy, self.angvel, self.servo]
        self.motor_pub.publish(msg)
        #self.get_logger().info('I move fw : "%f"' % msg.data[0])
        #self.get_logger().info('I move sw : "%f"' % msg.data[1])
        #self.get_logger().info('I rotate: "%f"' % msg.data[2])

    def audio_publisher(self, audio_num):
        if (self.sound_played!= audio_num):
            self.sound_played = audio_num
            msg = Int32()
            msg.data = audio_num
            self.audio_pub.publish(msg)
            self.get_logger().info('send audio: "%f"' % msg.data)       
    '''    def motor_publisher(self):
        msg = Float32MultiArray()
        msg.data = [self.speedx, self.speedy, self.angvel, self.servo]
        self.motor_pub.publish(msg)
        #self.get_logger().info('I move fw : "%f"' % msg.data[0])
        #self.get_logger().info('I move sw : "%f"' % msg.data[1])
        #self.get_logger().info('I rotate: "%f"' % msg.data[2])
    def servo_publisher(self, mode):
        msg = Int32()
        msg.data = mode
        self.servo_pub.publish(msg)'''
        
    '''def distance_callback(self, msg):
        if(len(msg.data) >= 3):
            try:
                self.distance1 = msg.data[0]
            except:
                pass
            try:
                self.distance2 = msg.data[1]
            except:
                pass
            try:
                self.distance3 = msg.data[2]
            except:
                pass'''

    '''def aruco_callback(self, msg):
        self.AruCo = msg.data
        if 13 in msg.data:
            self.AruCo_Color = "Blue"
        if 12 in msg.data:
            self.AruCo_Color = "Green"
        if 11 in msg.data:
            self.AruCo_Color = "Red"'''

    '''def color_callback(self, msg):
        self.color = msg.data'''

def main(args=None):
    eventlet.sleep(1)
    rclpy.init(args=args)

    mainAlg = MainAlg()
    rclpy.spin(mainAlg)

    mainAlg.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

