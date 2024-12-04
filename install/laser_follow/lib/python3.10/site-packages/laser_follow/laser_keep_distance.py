import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray


class Laser_Publisher(Node):

    def __init__(self):
        super().__init__("barrel_publisher")
        self.publisher_ = self.create_publisher(Int32MultiArray, "barrel_topic", 10)

    def detector(self):
        cap = cv2.VideoCapture(0) #check the port of the camera
        cap.set(cv2.CAP_PROP_FPS, 30)
        msg = Int32MultiArray()
        ret, frame = cap.read()
        frame_width = frame.shape[1] #w:image-width and h:image-height
        frame_height = frame.shape[0] 
        hsvFrame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV) # convert to hue - saturation - value
        while True:
            iSee = []
            ret, frame = cap.read()
                # Convert the frame to HSV color space
            hsvFrame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            # Set range for red color and define mask
            red_lower = np.array([136, 87, 111], np.uint8)
            red_upper = np.array([180, 255, 255], np.uint8)
            red_mask = cv2.inRange(hsvFrame, red_lower, red_upper)

            # Morphological Transform, Dilation for each color
            kernal = np.ones((5, 5), "uint8")

            # For red color
            red_mask = cv2.dilate(red_mask, kernal)

            # Finding contours for red color
            contours, _ = cv2.findContours(red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            if (contours):
                laser_contour = contours[0]
                for con in contours:
                    if cv2.contourArea(con)> cv2.contourArea(laser_contour):
                        laser_contour = con
                      
                M_red = cv2.moments(laser_contour)
                cx_red = int(M_red['m10'] / M_red['m00'])
                cy_red = int(M_red['m01'] / M_red['m00'])
                cv2.circle(frame, (cx_red, cy_red), 5, (0, 0, 255), -1)
                cv2.drawContours(frame, contours, -1, (0,255,0), 3)
                x_Diff_red = int(cx_red - frame_width/2)
                y_Diff_red = int(cy_red - frame_height/2)
                msg.data.insert(0,x_Diff_red)
                msg.data.insert(1,y_Diff_red)
                self.publisher_.publish(msg)
                self.get_logger().info('Distance from red x "%d"' % x_Diff_red)
                self.get_logger().info('Distance from red y "%d"' % y_Diff_red)
                #self.get_logger().info('area"%d"' % cv2.contourArea(laser_contour))
            else:
                msg.data.insert(0,-1000)
                msg.data.insert(1,-1000)
                self.publisher_.publish(msg)
            #cv2.imshow("Laser_pointer", frame)
            if cv2.waitKey(10) & 0xFF == ord('q'):
                cap.release()
                cv2.destroyAllWindows()
                break


def main(args=None):
    rclpy.init(args=args)
    Barrel = Laser_Publisher()
    Barrel.detector()
    Barrel.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


