import rclpy
from rclpy.node import Node

from std_msgs.msg import String #ボタン

import time
import Jetson.GPIO as GPIO

GPIO.setmode(GPIO.BOARD)
GPIO.setup(26, GPIO.IN)

class ButtonPublisher(Node):
    def __init__(self):
        super().__init__('button_publisher')
        self.pub= self.create_publisher(String, '/Button_pushed', 10)
        self.i=0
        self.button_callback()
    
    def button_callback(self):
        msg = String()
        try:
            while True:
                msg.data = 'Button Pushed: %d' % self.i
                GPIO.wait_for_edge(26, GPIO.FALLING)
                self.pub.publish(msg)
                time.sleep(0.3)
        except KeyboardInterrupt:
            pass
        finally:
            GPIO.cleanup()

def main(args=None):
    rclpy.init(args=args)
    button_publisher = ButtonPublisher()
    rclpy.spin(button_publisher)
    GPIO.cleanup()
    button_publisher.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()