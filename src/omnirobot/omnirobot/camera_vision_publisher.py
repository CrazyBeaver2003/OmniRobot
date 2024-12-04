import cv2
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32


class CameraPublisher(Node):
    def __init__(self):
        super().__init__("camera_publisher")
        self.publisher_ = self.create_publisher(Int32, "topic", 10)
    
    def is_red(self):
        camera = cv2.VideoCapture(0)
        msg = Int32()
        
        while True:
            iSee = False
            success, frame = camera.read()
            if success:
                hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
                binary = cv2.inRange(hsv, (170, 15, 169), (185, 255, 255))

                contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL,
                                                cv2.CHAIN_APPROX_NONE)

                if len(contours) != 0:
                    maxc = max(contours, key=cv2.contourArea)
                    moments = cv2.moments(maxc)

                    if moments["m00"] > 500:
                        cx = int(moments["m10"] / moments["m00"])
                        cy = int(moments["m01"] / moments["m00"])
                        iSee = True
            msg.data = int(iSee)
            self.get_logger().info('Publish: "%d"' % msg.data)
            self.publisher_.publish(msg)
                    

def main(args = None):
    rclpy.init(args=args)

    camera_publisher = CameraPublisher()
    camera_publisher.is_red()
    camera_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
