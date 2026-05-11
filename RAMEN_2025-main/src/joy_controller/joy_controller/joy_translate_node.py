import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

r_agv = 0.15
d_wheel = 0.125
rpm_max = 140
max_speed = (rpm_max/60)*d_wheel*3.14

class JoyTranslate(Node): 
    def __init__(self):
        super().__init__('joy_translate_node') 
        self.publisher = self.create_publisher(Twist,'cmd_vel', 10)
        self.subscription = self.create_subscription(Joy,'/joy', self.listener_callback, 10)
        self.vel = Twist()

    def listener_callback(self, Joy): 
        self.vel.linear.x  = Joy.axes[1] * max_speed  #[m/s]
        self.vel.angular.z  = Joy.axes[3] * max_speed/r_agv #[rad/s]
        self.publisher.publish(self.vel)
        #self.get_logger().info(f"Velocity: Linear={Joy.axes[1]}, Anguler={Joy.axes[3]}") 

def main(args=None):
    rclpy.init(args=args)
    joy_translate = JoyTranslate()
    rclpy.spin(joy_translate)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
