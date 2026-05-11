import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import Jetson.GPIO as GPIO
from threading import Thread
import time
from threading import Thread, Lock
from jetson_motor import (
    IN1, IN2, ENA, IN3, IN4, ENB,
    WHEEL_RADIUS, WHEEL_BASE, MAX_SPEED
)

from status_manager import NAME

# BOARDモードでのエンコーダピン番号（物理ピン）
ENC_A_L = 22  #
ENC_B_L = 24  #
ENC_A_R = 21  #
ENC_B_R = 23  #

class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller_node')
        self.moving = False
        self.use_smoothing = False
        self.status = 0  # 0:停止, 1:移動

        self.smoothing_timer = 0.0
        # GPIO初期化
        GPIO.setmode(GPIO.BOARD)
        for pin in (IN1, IN2, ENA, IN3, IN4, ENB):
            GPIO.setup(pin, GPIO.OUT)
        for pin in (ENC_A_L, ENC_B_L, ENC_A_R, ENC_B_R):
            GPIO.setup(pin, GPIO.IN)

        self.pwm_left = GPIO.PWM(ENA, 500)
        self.pwm_right = GPIO.PWM(ENB, 500)
        self.pwm_left.start(0)
        self.pwm_right.start(0)

       # 各モーターのログ出力タイミング管理
        self.last_motor_log_time = {"L": 0.0, "R": 0.0}
        self.motor_log_interval = 0.1  # 秒
        self.last_log_time = 0.0
        self.log_interval = 0.1

        # 目標RPM
        self.target_rpm = {"L": 0.0, "R": 0.0}

        # PID制御変数
        self.pid_error = {"L": 0.0, "R": 0.0}
        self.pid_integral = {"L": 0.0, "R": 0.0}
        self.pid_prev_error = {"L": 0.0, "R": 0.0}

       # 平滑化用前回値（初期化）
        self.smoothed_rpm = {"L": 0.0, "R": 0.0}
        self.rpm_alpha = 0.3  # 平滑化係数（0～1で設定、小さいほど滑らか）
        self.smoothed_target_rpm = {"L": 0.0, "R": 0.0}
        self.target_alpha = 0.3  # 目標に対するスムージング係数

        # PIDゲイン（チューニングは後で）
        self.Kp =  0.01 #0.012
        self.Ki =  0.015
        self.Kd =  0.004    #0.0041

        self.name = NAME


        self.value_old = {
            "L": 0.0,
            "R": 0.0
        }

        self.duty_old = {
            "L": 0.0,
            "R": 0.0,
        }

        # 定数読み込み
        self.wheel_radius = WHEEL_RADIUS
        self.wheel_base = WHEEL_BASE
        self.max_speed = MAX_SPEED
        self.max_duty_dif = 3  # 1%ずつ変化（スムージング）

        # エンコーダ関連
        self.encoder_count = {"L": 0, "R": 0}
        self.lock = Lock()

        # --- 割り込みによるエンコーダ読み取り設定 ---
        GPIO.remove_event_detect(ENC_A_L)
        GPIO.remove_event_detect(ENC_A_R)
        GPIO.add_event_detect(ENC_A_L, GPIO.RISING, callback=self.encoder_callback_L)
        GPIO.add_event_detect(ENC_A_R, GPIO.RISING, callback=self.encoder_callback_R)
        self.get_logger().info(f'status={self.status}')
        self.last_enc_count_L = 0
        self.last_enc_count_R = 0

        # エンコーダ監視スレッド
        Thread(target=self.rpm_logger_loop, daemon=True).start()

        # Twist購読
        self.subscription = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_callback, 10
        )

        self.subscription = self.create_subscription(
            String, '/start', self.start_callback, 10
        )

        self.subscription = self.create_subscription(
            String, '/stop', self.stop_callback, 10
        )

    def get_wheel_rpm(self,dt):
        pulse_per_rev = 64*70/4  # CPR: 64（モータ軸） × 70（ギア比） × 4分割（1逓倍なら1）
        with self.lock:
            current = self.encoder_count.copy()
        delta_l = current["L"] - self.last_enc_count_L
        delta_r = current["R"] - self.last_enc_count_R
        self.last_enc_count_L = current["L"]
        self.last_enc_count_R = current["R"]

        rev_l = delta_l / pulse_per_rev
        rev_r = delta_r / pulse_per_rev
        rpm_l = (rev_l / dt) * 60.0
        rpm_r = (rev_r / dt) * 60.0

        return rpm_l, rpm_r

    def rpm_logger_loop(self):
        last_time = time.perf_counter()
        rpm_max = self.max_speed / (2 * np.pi * self.wheel_radius) * 60  # m/s → rpm換算
        while True:
            now = time.time()
            dt = now - last_time
            last_time = now

            rpm_l, rpm_r = self.get_wheel_rpm(dt)
            current_rpm = {"L": rpm_l, "R": rpm_r}

            value_dict = {}  # L/R の value を保存しておく

            for side in ["L", "R"]:
                self.smoothed_rpm[side] = (
                     self.rpm_alpha * current_rpm[side] +
                     (1 - self.rpm_alpha) * self.smoothed_rpm[side]
                )

                self.smoothed_target_rpm[side] = (
                     self.target_alpha * self.target_rpm[side] +
                     (1 - self.target_alpha) * self.smoothed_target_rpm[side]
                )
                rpm = self.smoothed_rpm[side]
                target = self.smoothed_target_rpm[side]
                error = target - rpm
                self.pid_integral[side] += error * dt
                derivative = (error - self.pid_prev_error[side]) / dt
                self.pid_prev_error[side] = error

                correction = (
                    self.Kp * error +
                    self.Ki * self.pid_integral[side] +
                    self.Kd * derivative
                )

                # --- フィッティング関数による変換 ---
                rpm_ratio = self.smoothed_target_rpm[side] / rpm_max
                if self.use_smoothing:
                    base_value = self.fitting_with_launch(rpm_ratio, side)
                else:
                    base_value = self.fitting_without_launch(rpm_ratio, side)

                base_value = min(base_value, 0.95)
                value = base_value + correction
                value = max(min(value, 1.0), 0.0)
                value_dict[side] = value  # 保存


                # --- 変化量制限（value_oldとの比較） ---
                delta_value = value - self.value_old[side]
                max_value_delta = 0.1
                if abs(delta_value) > max_value_delta:
                    value = self.value_old[side] + np.sign(delta_value) * max_value_delta

                value_dict[side] = value  # ログ用に保存


                if side == "L":
                    self.set_motor(IN1, IN2, self.pwm_left, value, side='L', smoothing=self.use_smoothing)
                else:
                    self.set_motor(IN3, IN4, self.pwm_right, value, side='R', smoothing=self.use_smoothing)

            # --- ログ出力（0.1秒おき、LとRを一括） ---
            if now - self.last_log_time >= self.log_interval:
                for side in ["L", "R"]:
                    rpm = self.smoothed_rpm[side]
                    target = self.smoothed_target_rpm[side]
                    value = value_dict[side]
                    v_target = (target / 60.0) * 2 * np.pi * self.wheel_radius
                    v_actual = (rpm / 60.0) * 2 * np.pi * self.wheel_radius
                  #  self.get_logger().info(
                  #      f"[{side}] target={v_target:.3f} m/s, actual={v_actual:.3f} m/s, value={value:.2f}, launch={self.use_smoothing}"
                  #  )
                self.last_log_time = now

            time.sleep(0.01)
    def cmd_vel_callback(self, msg: Twist):
        v = msg.linear.x / self.max_speed
        turn = msg.angular.z * self.wheel_base / self.max_speed
        if self.status == 1:
            if turn > 0:
               left  = v
               right = v + turn
            else:
               right = v
               left = v - turn
        else:
           left = 0
           right = 0

        if abs(v) > 0 or abs(turn) > 0:
            if not self.moving:
                 self.moving = True
                 self.smoothing_timer = time.time()
        else:
            self.moving = False

    # === 発進直後ならスムージング有効 ===
        elapsed = time.time() - self.smoothing_timer if self.moving else 0.0
        self.use_smoothing = self.moving and elapsed < 3  # 3秒間だけ有効

    # 実際の最大RPMを推定（無負荷140rpmなどに合わせて）
        rpm_max = self.max_speed / (2 * np.pi * self.wheel_radius) * 60  # m/s → rpm換算
        self.target_rpm["L"] = left * rpm_max
        self.target_rpm["R"] = right * rpm_max

    def set_motor(self, in_a, in_b, pwm, value, side='?', smoothing=True):
        GPIO.setmode(GPIO.BOARD)
        GPIO.setwarnings(False)

    # OUTPUTとして設定されていない可能性があるので再セット（保険）
        GPIO.setup(in_a, GPIO.OUT)
        GPIO.setup(in_b, GPIO.OUT)

        duty = min(abs(value) * 100, 100)
        last_value = self.value_old[side]
        last_duty = self.duty_old[side]

    # === スムージング有効時だけ変化制限 ===
        if smoothing:
            duty_dif = duty - last_duty
            if abs(duty_dif) > self.max_duty_dif:
                duty = last_duty + self.max_duty_dif * np.sign(duty_dif)
        self.duty_old[side] = duty
        self.value_old[side] = value
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

        # ログを0.1秒おきに制限
        now = time.time()
        if now - self.last_motor_log_time[side] >= self.motor_log_interval:
            #self.get_logger().info(f"[{side}] value={value:.2f}, duty={duty:.1f}%, smoothing={smoothing}")
            self.last_motor_log_time[side] = now



    def fitting_with_launch(self, rpm_ratio, side):
        if self.status == 0:
            return 0.0

        if side == "L":
             return -6.2004 * rpm_ratio**5 + 16.4118 * rpm_ratio**4 + -13.9659 * rpm_ratio**3 + 4.6462 * rpm_ratio**2 + -0.0070 * rpm_ratio + 0.1251
        else:
             return -9.0376 * rpm_ratio**5 + 24.6751 * rpm_ratio**4 + -22.6855 * rpm_ratio**3 + 8.4117 * rpm_ratio**2 + -0.5550 * rpm_ratio + 0.1251

    def fitting_without_launch(self, rpm_ratio, side):
        if self.status == 0:
            return 0.0

        if side == "L":
            return  -8.4580 * rpm_ratio**5 + 22.6784 * rpm_ratio**4 + -18.8164 * rpm_ratio**3 + 5.1912 * rpm_ratio**2 + 0.4955 * rpm_ratio + 0.0004
        else:
            return  -6.9641 * rpm_ratio**5 + 18.2500 * rpm_ratio**4 + -14.7406 * rpm_ratio**3 + 3.7002 * rpm_ratio**2 + 0.6867 * rpm_ratio + 0.0005

    def start_callback(self, msg: String):
        if msg.data == NAME:
            self.status = 1
            self.get_logger().info('Motor started')

    def stop_callback(self, msg: String):
        if msg.data == NAME:
            self.status = 0
            self.get_logger().info('Motor stopped')

    def destroy_node(self):
        self.pwm_left.stop()
        self.pwm_right.stop()
        GPIO.cleanup()
        super().destroy_node()


    def encoder_callback_L(self, channel):
        with self.lock:
            self.encoder_count["L"] += 1

    def encoder_callback_R(self, channel):
        with self.lock:
            self.encoder_count["R"] += 1

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

