import rclpy
from rclpy.node import Node
import os
from std_msgs.msg import Int32
class BluetoothSpeakerNode(Node):
    audio_num = -1
    def __init__(self):
        super().__init__('bluetooth_speaker_node')
        self.subscription = self.create_subscription(
             Int32,
            'robot_audio_topic',
            self.listener_callback,
            1)
        self.subscription 

        #self.create_timer(1, self.play_audio)

    def listener_callback(self, msg):
        self.audio_num = msg.data
        os.system(f"bluetoothctl connect 9A:60:1F:C5:B1:A4") 
        if (self.audio_num ==1):
            audio_files = "detected.mp3" 
            os.system(f"mpg123 -q {audio_files}")  
        elif (self.audio_num == 2):
            audio_files = "locked.mp3"
            os.system(f"mpg123 -q {audio_files}")
        elif (self.audio_num == 3):
            audio_files = "lost.mp3"
            os.system(f"mpg123 -q {audio_files}")
        elif (self.audio_num == 4):
            audio_files = "obstacle.mp3"
            os.system(f"mpg123 -q {audio_files}") 
        elif (self.audio_num == 5):
            audio_files = "capturing.mp3"
            os.system(f"mpg123 -q {audio_files}")              



def main(args=None):
    rclpy.init(args=args)
    bluetooth_speaker_node = BluetoothSpeakerNode()
    rclpy.spin(bluetooth_speaker_node)
    bluetooth_speaker_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()