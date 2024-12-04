import cv2
import cv2.aruco as aruco
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray

class DetectorPublisher(Node):
    def __init__(self):
        super().__init__("detector_publisher")
        self.publisher_ = self.create_publisher(Int32MultiArray, "aruco_topic", 1)

    def detector(self):
        cap = cv2.VideoCapture(0)
        cap.set(cv2.CAP_PROP_FPS, 30)
        msg = Int32MultiArray()
        ret, frame = cap.read()
        frame_width = frame.shape[1] #w:image-width and h:image-height
        frame_height = frame.shape[0]
        while True:
            iSee = []
            ret, frame = cap.read()

            # определение словаря маркеров ArUco
            aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)

            # параметры детектирования маркеров
            parameters = aruco.DetectorParameters_create()

            # детектирование маркеров во входном кадре
            corners, ids, rejectedImgPoints = aruco.detectMarkers(frame, aruco_dict, parameters=parameters)
            x_centerDiff = 1000
            # если маркеры обнаружены
            if ids is not None:
                # рисуем границы маркеров и выводим их идентификаторы
                frame = aruco.drawDetectedMarkers(frame, corners, ids)
                #maxc = max(frame, key=cv2.contourArea)
                #moments = cv2.moments(maxc)
                for i in range(len(ids)):
                    c = corners[i][0]
                    iSee.append(int(ids[i]))
                msg.data = iSee
                ''' 
                base id  = 41 
                first_barrel_id = 35
                second_barrel_id = 49
                third_barrel_id = 119
                '''
                if 41 in ids:
                    centerX = corners[0][0][0][0]+ corners[0][0][1][0]
                    x_centerDiff = int(centerX/2 - frame_width/2)
                    marker_corners = corners[0][0]

                    center_y = np.mean(marker_corners[:, 1])
                    y_centerDiff = int (center_y.astype(np.int)/2 - frame_height/2)
                    msg.data.insert(0, x_centerDiff)
                    msg.data.insert(1, 41)
                    msg.data.insert(2, y_centerDiff)
                    self.publisher_.publish(msg)
                    self.get_logger().info('Publish:  - 41 "%d"  ' % x_centerDiff)
                    self.get_logger().info('Publish:  - 41 "%d"  ' % y_centerDiff)
                elif 35 in ids:
                    centerX = corners[0][0][0][0]+ corners[0][0][1][0]
                    x_centerDiff = int(centerX/2 - frame_width/2)
                    msg.data.insert(0, x_centerDiff)
                    msg.data.insert(1, 35)
                    msg.data.insert(2, 1111)
                    self.publisher_.publish(msg)
                    self.get_logger().info('Publish:  - 35 "%d"' % x_centerDiff)
                elif 49 in ids:
                    centerX = corners[0][0][0][0]+ corners[0][0][1][0]
                    x_centerDiff = int(centerX/2 - frame_width/2)
                    msg.data.insert(0, x_centerDiff)
                    msg.data.insert(1, 49)
                    msg.data.insert(2, 1111)
                    self.publisher_.publish(msg)
                    self.get_logger().info('Publish:  - 49 "%d"' % x_centerDiff)
                elif 13 in ids:
                    centerX = corners[0][0][0][0]+ corners[0][0][1][0]
                    x_centerDiff = int(centerX/2 - frame_width/2)
                    msg.data.insert(0, x_centerDiff)
                    msg.data.insert(1, 119)
                    msg.data.insert(2, 1111)
                    self.publisher_.publish(msg)
                    self.get_logger().info('Publish:  - 13 "%d"' % x_centerDiff)    
                else:
                    msg.data = {1000,1000, 1000}
                    self.publisher_.publish(msg)
                    self.get_logger().info('Unknown marker detected!&')  
            else:
                msg.data = {1000,1000, 1000}
                self.publisher_.publish(msg)
                self.get_logger().info('Nothing detected!') 

            #cv2.imshow('frame', frame)

            # остановка при нажатии на клавишу 'q'
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        
        # освобождение ресурсов и закрытие окон
        cap.release()
        cv2.destroyAllWindows()


def main(args = None):
    rclpy.init(args = args)
    ArUco = DetectorPublisher()
    ArUco.detector()
    ArUco.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()    
