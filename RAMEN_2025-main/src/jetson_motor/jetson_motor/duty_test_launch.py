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
        super().__init__('duty_test_launch')

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
        pulse_per_rev = 64 * 70 / 4
        test_values = [0.23,0.24,0.25,0.26,0.27,0.28,0.30,0.31,0.32,0.33,0.34,0.35,0.36,0.37,0.38]
         #[0.15, 0.18, 0.22, 0.25, 0.28, 0.3, 0.33, 0.35, 0.43, 0.45, 0.5, 0.6, 0.8, 1.0]
        record_duration = 3.0
        discard_time = 0.5
        interval = 0.1
        num_steps = int((record_duration - discard_time) / interval)

        for value in test_values:
            self.encoder_count = {"L": 0, "R": 0}
            self.set_motor(IN1, IN2, self.pwm_left, value)
            self.set_motor(IN3, IN4, self.pwm_right, value)

            time.sleep(discard_time)  # 最初の0.5秒は捨てる

            rpm_log = {"L": [], "R": []}
            for _ in range(num_steps):
                prev_count = self.encoder_count.copy()
                time.sleep(interval)
                curr_count = self.encoder_count.copy()

                for side in ("L", "R"):
                    delta = curr_count[side] - prev_count[side]
                    revs = delta / pulse_per_rev
                    rpm = (revs / interval) * 60.0
                    rpm_log[side].append(rpm)

        # モーター停止
            self.set_motor(IN1, IN2, self.pwm_left, 0.0)
            self.set_motor(IN3, IN4, self.pwm_right, 0.0)

            avg_rpm = {side: np.mean(rpm_log[side]) for side in ("L", "R")}
            speed = {
                side: (avg_rpm[side] / 60.0) * 2 * np.pi * self.wheel_radius
                for side in ("L", "R")
            }

            self.get_logger().info(
                f"value={value:.2f}, avg over {record_duration - discard_time:.1f}s | "
                f"[L] rpm={avg_rpm['L']:.1f}, speed={speed['L']:.3f} m/s | "
                f"[R] rpm={avg_rpm['R']:.1f}, speed={speed['R']:.3f} m/s"
            )

            time.sleep(3.0)  # クールダウン

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
