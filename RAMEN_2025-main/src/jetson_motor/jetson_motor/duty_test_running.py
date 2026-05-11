import rclpy
from rclpy.node import Node
import Jetson.GPIO as GPIO
import time
import numpy as np
from jetson_motor import IN1, IN2, ENA, IN3, IN4, ENB, WHEEL_RADIUS

# エンコーダA相ピン（BOARD番号）
ENC_A_L = 22
ENC_A_R = 21

class DutyTestNode(Node):
    def __init__(self):
        super().__init__('duty_test_running')

        GPIO.setmode(GPIO.BOARD)
        GPIO.setwarnings(False)

        # 出力ピン設定
        for pin in (IN1, IN2, IN3, IN4, ENA, ENB):
            GPIO.setup(pin, GPIO.OUT)

        # エンコーダ入力設定
        GPIO.setup(ENC_A_L, GPIO.IN)
        GPIO.setup(ENC_A_R, GPIO.IN)

        # PWM初期化
        self.pwm_left = GPIO.PWM(ENA, 500)
        self.pwm_right = GPIO.PWM(ENB, 500)
        self.pwm_left.start(0)
        self.pwm_right.start(0)

        self.wheel_radius = WHEEL_RADIUS
        self.encoder_count = {"L": 0, "R": 0}

        # 割り込み設定
        GPIO.remove_event_detect(ENC_A_L)
        GPIO.remove_event_detect(ENC_A_R)
        GPIO.add_event_detect(ENC_A_L, GPIO.RISING, callback=self.encoder_callback_l)
        GPIO.add_event_detect(ENC_A_R, GPIO.RISING, callback=self.encoder_callback_r)

        self.run_test()

    def encoder_callback_l(self, channel):
        self.encoder_count["L"] += 1

    def encoder_callback_r(self, channel):
        self.encoder_count["R"] += 1

    def run_test(self):
        pulse_per_rev = 64*70/4
        test_values = [0.25,0.28,0.3,0.33,0.35,0.43,0.45,0.48,0.5,0.55,0.6,0.8,1.0]


        self.set_motor(IN1, IN2, self.pwm_left, 0.25)
        self.set_motor(IN3, IN4, self.pwm_right, 0.25)
        time.sleep(2.0)
        for value in test_values:

            time.sleep(0.3)


            self.encoder_count = {"L": 0, "R": 0}

            self.set_motor(IN1, IN2, self.pwm_left, value)
            self.set_motor(IN3, IN4, self.pwm_right, value)

            start = time.perf_counter()
            time.sleep(1.0)
            end = time.perf_counter()

            dt = end - start

            log = []
            for side in ("L", "R"):
                count = self.encoder_count[side]
                revs = count / pulse_per_rev
                rpm = (revs / dt) * 60.0
                speed = (rpm / 60.0) * 2 * np.pi * self.wheel_radius

                log.append(
                    f"[{side}] count={count}, rpm={rpm:.1f}, speed={speed:.3f} m/s"
                )

            self.get_logger().info(
                f"value={value:.2f}, dt={dt:.3f}s | " + " | ".join(log)
            )

          

        GPIO.cleanup()
        self.get_logger().info("Test complete")
        rclpy.shutdown()

    def set_motor(self, in_a, in_b, pwm, value):
        duty = min(abs(value) * 100, 100)
        pwm.ChangeDutyCycle(duty)
        GPIO.output(in_a, GPIO.HIGH)
        GPIO.output(in_b, GPIO.LOW)

def main(args=None):
    rclpy.init(args=args)
    node = DutyTestNode()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
