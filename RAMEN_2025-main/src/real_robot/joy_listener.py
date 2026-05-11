import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import Jetson.GPIO as GPIO
from jetson_motor import IN1, IN2, ENA, IN3, IN4, ENB

class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller_node')

        GPIO.setmode(GPIO.BOARD)
        for pin in (IN1, IN2, ENA, IN3, IN4, ENB):
            GPIO.setup(pin, GPIO.OUT)

        self.pwm_left = GPIO.PWM(ENA, 500)
        self.pwm_right = GPIO.PWM(ENB, 500)
        self.pwm_left.start(0)
        self.pwm_right.start(0)

        self.subscription = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_callback, 10
        )

    def cmd_vel_callback(self, msg: Twist):
        speed = msg.linear.x
        turn = msg.angular.z

        # 左右の速度差をつける（差動駆動）
        left = speed - turn
        right = speed + turn

        self.set_motor(IN1, IN2, self.pwm_left, left, side='L')
        self.set_motor(IN3, IN4, self.pwm_right, right, side='R')

    def set_motor(self, in_a, in_b, pwm, value, side='?'):
        duty = min(abs(value) * 100, 100)
        pwm.ChangeDutyCycle(duty)
        if value > 0:
            GPIO.output(in_a, GPIO.HIGH)
            GPIO.output(in_b, GPIO.LOW)
        elif value < 0:
            GPIO.output(in_a, GPIO.LOW)
            GPIO.output(in_b, GPIO.HIGH)
        else:
            GPIO.output(in_a, GPIO.LOW)
            GPIO.output(in_b, GPIO.LOW)


        self.get_logger().info(f"[{side}] speed={value:.2f} → duty={duty:.1f}%")

    def destroy_node(self):
        self.pwm_left.stop()
        self.pwm_right.stop()
        GPIO.cleanup()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = MotorController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()