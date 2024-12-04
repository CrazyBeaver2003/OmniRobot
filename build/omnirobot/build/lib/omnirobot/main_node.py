import asyncio
import eventlet
import rclpy
from rclpy.node import Node

from time import sleep
from std_msgs.msg import Int32
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import String

class MainAlg(Node):
    __flag = 1                                                                                                                                                      ################################## Можно Менять На Поле Для Отладки  ######################################
    __AruCoFlag = 0
    __TipTap = 0
    __Blue = 0
    tmp_distance = 0
    distance1 = 0
    distance2 = 0
    distance3 = 0
    AruCo = [-1,-1]
    AruCo_Color = "None"
    AruCoColorFlag = "None"
    color = "None"

    def __init__(self):

        super().__init__('MainAlg')
        self.distance_color_pub = self.create_publisher(Int32, 'distance_color_topic', 10)
        self.motor_pub = self.create_publisher(Int32MultiArray, 'motor_control_topic', 10)
        self.servo_pub = self.create_publisher(Int32, 'servo_control_topic', 10)
        self.distance_sub = self.create_subscription(Int32MultiArray, 'distance_topic', self.distance_callback, 10)
        self.color_sub = self.create_subscription(String, 'color_topic', self.color_callback, 10)
        self.aruco_sub = self.create_subscription(Int32MultiArray, 'aruco_topic', self.aruco_callback, 10)

        timer_period = 0.1    # seconds
        self.algorithm_cycle = self.create_timer(timer_period, self.main_algorithm)

    def main_algorithm(self):
        ################################################################# 1 Flag #################################################################
        if (self.__flag == 1):
            self.get_logger().info('MISSION FLAG: "%d"' % self.__flag)
            self.CD_publisher(0)
            if (self.AruCo[0] == 0  or self.AruCo[0] == 2) or (90 < self.distance2 <110):                                                                       ################################## Поменять На 86 - 94  ######################################
                if(self.__TipTap == 0):
                    self.__flag = 2
                    return
                else:
                    self.__flag = 10
                    return
            if (40 > self.distance1 > 13):                                                                                                                                ################################## Поменять На 5 ######################################
                self.motor_publisher(7, 6)
                return
            self.motor_publisher(4, 4)

        ################################################################# 2 Flag #################################################################
        if (self.__flag == 2):
            self.get_logger().info('MISSION FLAG: "%d"' % self.__flag)
            self.CD_publisher(0)
            if self.AruCo[0] == 0  or self.AruCo[0] == 2:
    
                if self.AruCo[1] < -15:
                    self.motor_publisher(abs(int(self.AruCo[1]/26)),3)
                    return
                elif self.AruCo[1] > 15:
                    self.motor_publisher(abs(int(self.AruCo[1]/26)),4)
                    return
                else:
                    self.motor_publisher(0,0)
                    self.__flag = 3
                    return
            if(self.distance2 > 50):                                                                                                                                 ################################## Поменять На 60  ######################################
                self.motor_publisher(3,3)
                return
            else:
                self.motor_publisher(0, 0)
                self.__flag = 1
                return

        ################################################################# 3 Flag #################################################################
        if (self.__flag == 3):asyncio
            self.get_logger().info('MISSION FLAG: "%d"' % self.__flag)
            self.get_logger().info('DISTANCE: "%d"' % self.distance1)
            if self.AruCo[0] == 0  or self.AruCo[0] == 2:
                if abs(self.AruCo[1]) > 17:
                    self.__flag = 2
                    return                                       #Вынести Этот Блок Кода В Функцию
            if self.distance1 > 30:                                                                                                                                     ################################## Поменять На 34  ######################################             
                self.motor_publisher(0, 0)
                self.CD_publisher(1)
                if self.distance1 < 40:                                                                                                                                 ################################## Поменять На 34  ######################################
                    self.CD_publisher(1)
                    self.motor_publisher(4, 5)
                    return
                else:
                    self.motor_publisher(0, 0)
                    eventlet.sleep(3)
                    self.CD_publisher(0)
                    self.__flag = 4
                    return
            self.CD_publisher(0)
            self.motor_publisher(4, 5)

        ################################################################# 4 Flag #################################################################
        if (self.__flag == 4):
            self.get_logger().info('MISSION FLAG: "%d"' % self.__flag)
            self.CD_publisher(0)
            self.motor_publisher(6, 6)
            if(10 < self.distance1 < 17):                                                                                                                                    ################################## Поменять На +-17  ######################################
                self.motor_publisher(0, 0)
                self.__flag = 5

        ################################################################# 5 Flag #################################################################
        if (self.__flag == 5):
            self.get_logger().info('MISSION FLAG: "%d"' % self.__flag)
            self.get_logger().info('DISTANCE: "%d"' % self.distance3)
            self.CD_publisher(0)
            if (self.distance1 < 17):
                self.motor_publisher(3, 5)
                return
            if(10 < self.distance3 < 19):                                                                                                                                    ################################## Поменять На +-17  ######################################
                self.motor_publisher(0, 0)
                self.__flag = 6
                return
            self.motor_publisher(7, 4)

        ################################################################# 6 Flag #################################################################
        if (self.__flag == 6):
            self.get_logger().info('MISSION FLAG: "%d"' % self.__flag)
            self.CD_publisher(0)
            if(self.distance3 < 16):                                                                                                                                        ################################## Поменять На +-90  ######################################
                self.motor_publisher(0, 0)
                eventlet.sleep(3)
                self.motor_publisher(4, 1)
                eventlet.sleep(9.1)
                self.motor_publisher(0, 0)
                self.CD_publisher(0)
                self.tmp_distance = self.distance1
                self.__flag = 7
                return
            self.motor_publisher(7, 4)

        ################################################################# 7 Flag #################################################################
        if (self.__flag == 7):
            self.get_logger().info('MISSION FLAG: "%d"' % self.__flag)
            self.motor_publisher(7, 6)
            self.CD_publisher(0)
            if (self.distance2 > 13):
                self.motor_publisher(4, 3)
                return
            if (20 > self.distance1 > 15) and (self.distance1 != self.tmp_distance):     
                self.__flag = 8
                return
            
        ################################################################# 8 Flag #################################################################
        if (self.__flag == 8):
            self.get_logger().info('MISSION FLAG: "%d"' % self.__flag)
            self.CD_publisher(0)
            if(15 > self.distance1):
                self.motor_publisher(7, 5)
                return
            
            self.motor_publisher(7, 4)
            if (self.color == "Blue"):
                self.motor_publisher(0, 0)
                self.__flag = 9
                return
            elif((self.AruCo[0] == 0  or self.AruCo[0] == 2) or (55 < self.distance2 < 65)):                                                                                       ################################## Поменять На 86 - 94  ######################################
                self.motor_publisher(0, 0)
                self.__flag = 10
                
        ################################################################# 9 Flag ################################################################# FOR BLUE
        if (self.__flag == 9):
            self.get_logger().info('MISSION FLAG: "%d"' % self.__flag)
            self.CD_publisher(0)
            if (self.distance1 < 15):
                self.motor_publisher(4, 5)
                return
            if(self.color == "Blue"):
                self.motor_publisher(3, 4)
                self.__Blue = 1
                return
            elif (self.__AruCoFlag == 0):
                self.motor_publisher(3, 3)
                eventlet.sleep(1)
                self.motor_publisher(0, 0)
                eventlet.sleep(3)
                self.__flag = 10             
                return
            else:
                self.motor_publisher(3, 3)
                eventlet.sleep(0.7)
                self.motor_publisher(0, 0)
                eventlet.sleep(3)
                self.__flag = 13
                return
            
        ################################################################# 10 Flag ################################################################# 
        if (self.__flag == 10):
            self.get_logger().info('MISSION FLAG: "%d"' % self.__flag)
            self.CD_publisher(0)
            if((self.AruCo[0] == 0  or self.AruCo[0] == 2)):
                if self.AruCo[1] <-10:
                    self.motor_publisher(abs(int(self.AruCo[1]/28)),3)
                elif self.AruCo[1] >10:
                    self.motor_publisher(abs(int(self.AruCo[1]/28)),4)
                else:
                    self.motor_publisher(0,0)
                    self.__flag = 11
                    return
            elif (self.distance2 > 70):                                                                                                                                                 ################################## Поменять На 86 - 94  ######################################
                self.motor_publisher(3, 3)
                return
            else:
                self.__TipTap = 1
                self.__flag = 1
                return
            
        ################################################################# 11 Flag ################################################################# 
        if (self.__flag == 11):
            self.get_logger().info('MISSION FLAG: "%d"' % self.__flag)
            self.get_logger().info('DISTANCE: "%d"' % self.distance1)
            if self.AruCo[0] == 0  or self.AruCo[0] == 2:
                if abs(self.AruCo[1]) > 12:
                    self.__flag = 10
                    return
            # if self.distance1 > 90:    
            #     self.motor_publisher(0, 0)                                                                                                                                                  ################################## Поменять На +-35  ######################################
            #     self.CD_publisher(1)
            #     eventlet.sleep(0.3)
            #     self.CD_publisher(1)
            #     eventlet.sleep(0.3)
            #     self.CD_publisher(1)
            #     eventlet.sleep(0.3)
            #     self.CD_publisher(1)
            #     eventlet.sleep(0.3)
            #     self.motor_publisher(4, 6)
            #     eventlet.sleep(1.7)
            #     self.motor_publisher(4, 1)
            #     eventlet.sleep(9)
            #     self.motor_publisher(0, 0)
            #     self.__flag = 12
            #     return
            else:
                self.motor_publisher(6, 5)
                eventlet.sleep(3)
                self.CD_publisher(1)
                eventlet.sleep(1)
                self.motor_publisher(5, 6)
                eventlet.sleep(1.2)
                self.motor_publisher(4, 1)
                eventlet.sleep(9)
                self.motor_publisher(0, 0)
                self.__flag = 12
                self.motor_publisher(4, 6)
                eventlet.sleep(3.5)  
                self.CD_publisher(1)      
                return
            
            self.motor_publisher(6, 5)
            self.CD_publisher(0)
        
        ################################################################# 12 Flag ################################################################# 
        if (self.__flag == 12):
            self.get_logger().info('MISSION FLAG: "%d"' % self.__flag)
            self.CD_publisher(1)                                                                                                                                                ################################## Отрегулировать  ######################################
            # if(self.distance3 < 71):
            #     self.motor_publisher(3, 3)
            #     return
            # if(self.distance2 < 72):
            #     self.motor_publisher(3, 4)
            #     return
            self.motor_publisher(4, 5)
            eventlet.sleep(6.3)                                                                                                                                                        ################################## Отрегулировать  ######################################
            self.CD_publisher(0)
            self.motor_publisher(4, 6)
            eventlet.sleep(2)                                                                                                                                                        ################################## Отрегулировать  ######################################
            self.__AruCoFlag = 1
            self.__flag = 13
            return
        
        ################################################################# 13 Flag ################################################################# 
        if (self.__flag == 13):
            self.get_logger().info('MISSION FLAG: "%d"' % self.__flag)
            self.CD_publisher(0)
            if ((self.color == "Blue") and (self.__Blue == 0)):
                self.__flag = 9
                return
            self.motor_publisher(4,3)        
            if(self.distance2 < 19):                                                                                                                                          ################################## Сделать Распознавание Цвета Через Маркер И ТД ######################################
                self.motor_publisher(0, 0)
                self.AruCo_Color = "None"
                self.__flag = 14
                return
            
        ################################################################# 14 Flag #################################################################
        if (self.__flag == 14):
            self.get_logger().info('MISSION FLAG: "%d"' % self.__flag)
            self.get_logger().info('ARUCO COLOR: "%s"' % self.AruCo_Color)
            self.CD_publisher(0)
            if (self.AruCo_Color != "None"):
                if (self.distance1 < 80):
                    self.motor_publisher(7, 5)
                    return 
                else:
                    self.motor_publisher(4, 1)
                    eventlet.sleep(9.05)
                    self.motor_publisher(6, 4)
                    eventlet.sleep(3)
                    self.motor_publisher(0, 0)
                    self.AruCoColorFlag = self.AruCo_Color
                    self.__flag = 15
                    return

        ################################################################# 15 Flag #################################################################
        if (self.__flag == 15):
            self.get_logger().info('MISSION FLAG: "%d"' % self.__flag)
            self.get_logger().info('ARUCO COLOR: "%s"' % self.AruCo_Color)
            self.CD_publisher(0)
            if (self.AruCo[0] == 0  or self.AruCo[0] == 2) or (60 > self.distance2 > 45):                                                                       ################################## Поменять На 86 - 94  ######################################
                self.__flag = 16
                return
            if (self.distance1 > 14):                                                                                                                                ################################## Поменять На 5 ######################################
                self.motor_publisher(5, 6)
                return
            elif (self.distance1 < 10):
                self.motor_publisher(5, 5)
                return
            self.motor_publisher(4, 3)
            
        ################################################################# 16 Flag #################################################################
        if (self.__flag == 16):
            self.get_logger().info('MISSION FLAG: "%d"' % self.__flag)
            self.CD_publisher(0)
            if((self.AruCo[0] == 0  or self.AruCo[0] == 2)):
                if self.AruCo[1] < -12:
                    self.motor_publisher(abs(int(self.AruCo[1]/25)),3)
                    return
                elif self.AruCo[1] > 12:
                    self.motor_publisher(abs(int(self.AruCo[1]/25)),4)
                    return
                else:
                    self.motor_publisher(0,0)
                    self.__flag = 17
                    return
            elif (self.distance2 < 50):                                                                                                                                                 ################################## Поменять На 86 - 94  ######################################
                self.motor_publisher(3, 4)
                return
            else:
                self.__flag = 15
                return
            
        ################################################################# 17 Flag #################################################################    
        if (self.__flag == 17):
            self.get_logger().info('MISSION FLAG: "%d"' % self.__flag) 
            if self.AruCo[0] == 0  or self.AruCo[0] == 2:
                if abs(self.AruCo[1]) > 12:
                    self.__flag = 16
                    return       
            if (self.distance1 > 30):
                self.motor_publisher(4, 5)
                eventlet.sleep(1.5)
                self.CD_publisher(1)
                eventlet.sleep(1)
                self.motor_publisher(4, 6)
                eventlet.sleep(1.5)
                self.__flag = 18
                return
            self.motor_publisher(4, 5)    
            self.CD_publisher(0)
        ################################################################# 18 Flag #################################################################
        if (self.__flag == 18):
            self.get_logger().info('MISSION FLAG: "%d"' % self.__flag)
            self.CD_publisher(1)
            self.motor_publisher(4, 6)
            if (self.distance1 < 15):
                self.__flag = 19
                return
        
        ################################################################# 19 Flag #################################################################
        if (self.__flag == 19):
            self.get_logger().info('MISSION FLAG: "%d"' % self.__flag)
            self.CD_publisher(1)
            if (self.AruCoColorFlag == "Red"):
                self.motor_publisher(4, 4)
                if self.color != "Red":
                    self.motor_publisher(4, 3)
                    eventlet.sleep(1)
                    self.motor_publisher(0 ,0)
                    self.__flag = 20
                    return
            elif (self.AruCoColorFlag == "Green"):
                self.motor_publisher(4, 3)
                if self.color == "Blue":
                    self.motor_publisher(4, 4)
                    eventlet.sleep(4.2)
                    self.motor_publisher(0 ,0)
                    self.__flag = 20
                    return    
            elif (self.AruCoColorFlag == "Blue"):
                self.motor_publisher(4, 3)
                if self.color == "Blue":
                    self.motor_publisher(4, 3)
                    eventlet.sleep(0.7x)
                    self.motor_publisher(0 ,0)
                    self.__flag = 20
                    return

        ################################################################# 20 Flag #################################################################
        if (self.__flag == 20):
            self.get_logger().info('MISSION FLAG: "%d"' % self.__flag)
            self.CD_publisher(1)


    def CD_publisher(self, servo_mess):
        msg = Int32()
        msg.data = servo_mess
        self.distance_color_pub.publish(msg)

    def motor_publisher(self, speed, mode):
        msg = Int32MultiArray()
        msg.data = [speed, mode]
        self.motor_pub.publish(msg)
        #self.get_logger().info('I HEARD: "%d"' % self.distance[0])

    def servo_publisher(self, mode):
        msg = Int32()
        msg.data = mode
        self.servo_pub.publish(msg)
        
    def distance_callback(self, msg):
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
                pass

    def aruco_callback(self, msg):
        self.AruCo = msg.data
        if 13 in msg.data:
            self.AruCo_Color = "Blue"
        if 12 in msg.data:
            self.AruCo_Color = "Green"
        if 11 in msg.data:
            self.AruCo_Color = "Red"

    def color_callback(self, msg):
        self.color = msg.data

def main(args=None):
    eventlet.sleep(7)
    rclpy.init(args=args)

    mainAlg = MainAlg()
    rclpy.spin(mainAlg)

    mainAlg.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()



