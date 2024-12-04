import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray


class Barrel_Publisher(Node):

    def __init__(self):
        super().__init__("barrel_publisher")
        self.publisher_ = self.create_publisher(Int32MultiArray, "barrel_topic", 10)

    def detector(self):
        cap = cv2.VideoCapture(0) #check the port of the camera
        cap.set(cv2.CAP_PROP_FPS, 30)
        msg = Int32MultiArray()
        ret, frame = cap.read()
        frame_width = frame.shape[1] #w:image-width and h:image-height
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

            # Set range for green color and define mask
            green_lower = np.array([25, 52, 72], np.uint8)
            green_upper = np.array([102, 255, 255], np.uint8)
            green_mask = cv2.inRange(hsvFrame, green_lower, green_upper)

            # Set range for blue color and define mask
            blue_lower = np.array([94, 80, 2], np.uint8)
            blue_upper = np.array([120, 255, 255], np.uint8)
            blue_mask = cv2.inRange(hsvFrame, blue_lower, blue_upper)

            # Morphological Transform, Dilation for each color
            kernal = np.ones((5, 5), "uint8")

            # For red color
            red_mask = cv2.dilate(red_mask, kernal)

            # For green color
            green_mask = cv2.dilate(green_mask, kernal)

            # For blue color
            blue_mask = cv2.dilate(blue_mask, kernal)

            # Finding contours for red color
            contours, _ = cv2.findContours(red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            if contours:
                largest_contour_red = max(contours, key=cv2.contourArea)
                M_red = cv2.moments(largest_contour_red)
                cx_red = int(M_red['m10'] / M_red['m00'])
                cy_red = int(M_red['m01'] / M_red['m00'])
                cv2.circle(frame, (cx_red, cy_red), 5, (0, 0, 255), -1)
                x_Diff_red = int(cx_red - frame_width/2)
                msg.data.insert(1, x_Diff_red)
                msg.data.insert(2, 0)
                self.publisher_.publish(msg)
                self.get_logger().info('Distance from red"%d"' % x_Diff_red)

            # Finding contours for green color
            contours, _ = cv2.findContours(green_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            if contours:
                largest_contour_green = max(contours, key=cv2.contourArea)
                M_green = cv2.moments(largest_contour_green)
                cx_green = int(M_green['m10'] / M_green['m00'])
                cy_green = int(M_green['m01'] / M_green['m00'])
                cv2.circle(frame, (cx_green, cy_green), 5, (0, 255, 0), -1)
                x_Diff_green = int(cx_green - frame_width/2)
                msg.data.insert(1, x_Diff_green)
                msg.data.insert(2, 0)
                self.publisher_.publish(msg)
                self.get_logger().info('Distance from green%d"' % x_Diff_green)

            # Finding contours for blue color
            contours, _ = cv2.findContours(blue_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            if contours:
                largest_contour_blue = max(contours, key=cv2.contourArea)
                M_blue = cv2.moments(largest_contour_blue)
                cx_blue = int(M_blue['m10'] / M_blue['m00'])
                cy_blue = int(M_blue['m01'] / M_blue['m00'])
                cv2.circle(frame, (cx_blue, cy_blue), 5, (255, 0, 0), -1)
                x_Diff_blue= int(cx_blue - frame_width/2)
                msg.data.insert(1, x_Diff_blue)
                msg.data.insert(2, 0)
                self.publisher_.publish(msg)
                self.get_logger().info('Distance from blue%d"' % x_Diff_blue)
            # Program Termination
            #cv2.imshow("Multiple Color Detection in Real-TIme", frame)
            if cv2.waitKey(10) & 0xFF == ord('q'):
                cap.release()
                cv2.destroyAllWindows()
                break


def main(args=None):
    rclpy.init(args=args)
    Barrel = Barrel_Publisher()
    Barrel.detector()
    Barrel.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


