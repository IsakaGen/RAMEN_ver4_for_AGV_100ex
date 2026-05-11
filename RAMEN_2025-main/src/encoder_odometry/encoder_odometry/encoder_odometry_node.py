import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import Jetson.GPIO as GPIO
import numpy as np
import time
from jetson_motor import WHEEL_RADIUS

# GPIOピン定義（BOARD番号）
ENC_A_L = 22
ENC_A_R = 21

# ロボットパラメータ
WHEEL_BASE = 0.31  # 車軸長さ [m]
PULSE_PER_REV = 64 * 70 / 4  # エンコーダ1回転あたりパルス数

class EncoderOdometryNode(Node):
    def __init__(self):
        super().__init__('encoder_odometry_node')

        # GPIO初期化
        GPIO.setmode(GPIO.BOARD)
        GPIO.setwarnings(False)
        GPIO.setup(ENC_A_L, GPIO.IN)
        GPIO.setup(ENC_A_R, GPIO.IN)

        GPIO.remove_event_detect(ENC_A_L)
        GPIO.remove_event_detect(ENC_A_R)
        GPIO.add_event_detect(ENC_A_L, GPIO.RISING, callback=self.encoder_callback_l)
        GPIO.add_event_detect(ENC_A_R, GPIO.RISING, callback=self.encoder_callback_r)

        # 状態変数
        self.encoder_count = {"L": 0, "R": 0}
        self.prev_count = {"L": 0, "R": 0}
        self.prev_time = time.perf_counter()

        self.x = 0.0  # 位置
        self.y = 0.0
        self.yaw = 0.0

        self.odom_pub = self.create_publisher(Odometry, '/encoder_odom', 10)
        self.br = TransformBroadcaster(self)

        # タイマー（10Hz）
        self.create_timer(0.1, self.update_odometry)

    def encoder_callback_l(self, channel):
        self.encoder_count["L"] += 1

    def encoder_callback_r(self, channel):
        self.encoder_count["R"] += 1

    def update_odometry(self):
        now = self.get_clock().now().to_msg()
        current_time = time.perf_counter()
        dt = current_time - self.prev_time
        self.prev_time = current_time

        # 差分パルスから回転数 [rev/s]
        delta_L = self.encoder_count["L"] - self.prev_count["L"]
        delta_R = self.encoder_count["R"] - self.prev_count["R"]
        self.prev_count["L"] = self.encoder_count["L"]
        self.prev_count["R"] = self.encoder_count["R"]

        rev_L = delta_L / PULSE_PER_REV
        rev_R = delta_R / PULSE_PER_REV

        v_L = rev_L * 2 * np.pi * WHEEL_RADIUS / dt
        v_R = rev_R * 2 * np.pi * WHEEL_RADIUS / dt

        # 並進・回転速度
        v = (v_R + v_L) / 2.0
        omega = (v_R - v_L) / WHEEL_BASE

        # 位置更新
        dx = v * np.cos(self.yaw) * dt
        dy = v * np.sin(self.yaw) * dt
        dtheta = omega * dt

        self.x += dx
        self.y += dy
        self.yaw += dtheta

        # オドメトリメッセージ作成
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        # 位置
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0  # 明示的に設定

        # 姿勢（クォータニオン：Yawのみ）
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = np.sin(self.yaw / 2.0)
        odom.pose.pose.orientation.w = np.cos(self.yaw / 2.0)

        # 線形・角速度
        odom.twist.twist.linear.x = v
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.linear.z = 0.0
        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = omega

        # 共分散（自己位置推定で必要になる）
        odom.pose.covariance = [
               0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
               0.0, 0.01, 0.0, 0.0, 0.0, 0.0,
               0.0, 0.0, 999.0, 0.0, 0.0, 0.0,
               0.0, 0.0, 0.0, 999.0, 0.0, 0.0,
               0.0, 0.0, 0.0, 0.0, 999.0, 0.0,
               0.0, 0.0, 0.0, 0.0, 0.0, 0.05
        ]

        odom.twist.covariance = [
              0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
              0.0, 0.01, 0.0, 0.0, 0.0, 0.0,
              0.0, 0.0, 999.0, 0.0, 0.0, 0.0,
              0.0, 0.0, 0.0, 999.0, 0.0, 0.0,
              0.0, 0.0, 0.0, 0.0, 999.0, 0.0,
              0.0, 0.0, 0.0, 0.0, 0.0, 0.05
        ]
        # TFブロードキャスト
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation.z = np.sin(self.yaw / 2)
        t.transform.rotation.w = np.cos(self.yaw / 2)
        self.br.sendTransform(t)
        self.odom_pub.publish(odom)       
 
def main(args=None):
    rclpy.init(args=args)
    node = EncoderOdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        GPIO.cleanup()
        node.get_logger().info("Odometry node stopped")
    rclpy.shutdown()

if __name__ == '__main__':
    main()
