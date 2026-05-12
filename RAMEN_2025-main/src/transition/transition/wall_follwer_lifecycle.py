#!/usr/bin/env python3
# wall_follower_lifecycle.py
# 最初のカーブを曲がる距離を変更しました．209行目


import time
import rclpy
from rclpy.lifecycle import Node, TransitionCallbackReturn
from lifecycle_msgs.msg import State, Transition
from lifecycle_msgs.srv import ChangeState, GetState
from rclpy.node import Node as BasicNode
from std_msgs.msg import String
from functools import partial
from lifecycle_msgs.msg import State as LCState
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np
import math
import random


# ──────────────────────────────────────────────
# 1) 位置決めモード用 Lifecycle ノード
# ──────────────────────────────────────────────
class wallfollwerModeNode(Node):
    """
    * 起動直後に UNCONFIGURED → CONFIGURE → ACTIVATE まで
      自動で遷移して “壁追従” を開始する。
    * go_straight() が /scan 到着ごとに呼ばれ、
      元の WallFollower と機能が 100 % 一致する。
    """

    def __init__(self):
        super().__init__('wall_follower_lifecycle_node')
        
        # WallFollower パラメータ（オリジナルそのまま）

        self.segment = 1  # 1〜4：現在の直線区間番号
        self.start_time = None
        self.target_distance = 0.35
        self.prev_wall_angle = 0.0
        self.prev2_error = 0.0   # 2ステップ前の誤差
        self.error_hist = [0.0, 0.0, 0.0, 0.0]  # 4ステップ分（古→新）
        self.Kp = 0.2
        self.Kd = 0.1
        self.angle_gain = 0.3
        self.wall_angle_d_gain = 0.15
        self.prev_error = 0.0
        self.mode = "FOLLOWING"
        self.turn_start_time = 0.0
        self.pause_time = None

        # ── 1. コールバックグループを分離 ──
        self.io_group   = ReentrantCallbackGroup()          # サービス・軽い処理
        self.compute_g  = MutuallyExclusiveCallbackGroup()  # RANSAC 計算系
    # Lifecycle Callbacks
    # ------------------------------------------------------------------
    def on_configure(self, _state):
        """
        UNCONFIGURED → INACTIVE で一度だけ呼ばれる。
        ここで Publisher / Subscriber を作る。
        """
        
        self.get_logger().info('[LC] Configured')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state): #TransitionCallbackReturn.SUCCESS（inactive→active or configured→active）を受け取ったら発動．
        self.get_logger().info('[LC] ACTIVE – wall following ON')
        self.cmd_pub = self.create_lifecycle_publisher(Twist, 'cmd_vel', 10)
        # /scan が届くたび go_straight() を実行
        self.scan_sub = self.create_subscription(
            LaserScan, 'scan', self.go_straight, 10, callback_group=self.compute_g)
        # 旋回時にturningを発行
        self.turn_pub = self.create_publisher(String, 'turning', 10, callback_group=self.io_group)

        #self.cmd_pub.on_activate() 
        return super().on_activate(state)

    def on_deactivate(self, state): # TransitionCallbackReturn.SUCCESS（active→inactive）を受け取ったら発動．
        #self.cmd_pub.on_deactivate()
        stop = Twist()                     # All fields 0
        self.cmd_pub.publish(stop)
        self.get_logger().info('[LC] INACTIVE – wall following OFF')
        return super().on_deactivate(state)

    # ------------------------------------------------------------------
    # ★ 元 _announce_positioning_mode → go_straight ★
    #   オリジナル scan_callback を 100 % そのまま移植
    #   ※ /scan Subscriber から直接呼ばれるので
    #     引数 scan は LaserScan 型のまま
    # ------------------------------------------------------------------
    def go_straight(self, scan: LaserScan):
          # === 処理周期の計算 ===
        #if self.current_state.id != LCState.PRIMARY_STATE_ACTIVE:
            #return
          # === 処理周期の計算 ===
        #self.get_logger().info(f"Now Segment = {self.segment}")
        now = self.get_clock().now()
        if not hasattr(self, 'prev_time'):
            self.prev_time = now
            return

        dt = (now - self.prev_time).nanoseconds * 1e-9  # 秒に変換
        self.prev_time = now
        #self.get_logger().info(f"[周期] dt = {dt:.4f} 秒")

        ranges = np.array(scan.ranges)
        angle_min = scan.angle_min
        angle_inc = scan.angle_increment

        # 壁検出用RANSAC（右側）
        start_deg = 60
        end_deg = 120
        start_idx = int((math.radians(start_deg) - angle_min) / angle_inc)
        end_idx = int((math.radians(end_deg) - angle_min) / angle_inc)

        points = []
        for i in range(start_idx, end_idx):
            if i < 0 or i >= len(ranges):
                continue
            r = ranges[i]
            if not math.isfinite(r):
                continue
            angle = angle_min + i * angle_inc
            x = r * math.cos(angle)
            y = r * math.sin(angle)
            points.append([x, y])

        if len(points) < 2:
            self.get_logger().info("Not enough points for RANSAC")
            return

        # RANSAC直線フィッティング
        best_inliers = []
        best_model = (0, 0, 0)

        for _ in range(150):
            p1, p2 = random.sample(points, 2)
            a = p2[1] - p1[1]
            b = p1[0] - p2[0]
            c = -a * p1[0] - b * p1[1]
            norm = math.hypot(a, b)
            if norm == 0:
                continue
            inliers = []
            for x, y in points:
                dist = abs(a * x + b * y + c) / norm
                if dist < 0.08:
                    inliers.append((x, y))
            if len(inliers) > len(best_inliers):
                best_inliers = inliers
                best_model = (a, b, c)

        if not best_inliers:
            self.get_logger().info("No line found")
            return

        # 壁との距離と角度
        a, b, c = best_model
        distance = abs(c) / math.hypot(a, b)
        error = distance - self.target_distance
        d_error = error - self.prev_error
        dd_error = d_error - (self.prev_error - self.prev2_error)

        # 誤差履歴更新（古い順にずらす）
        self.error_hist = self.error_hist[1:] + [error]

        # 差分（1ステップ目と2ステップ目）
        d1 = self.error_hist[3] - self.error_hist[2]
        d2 = self.error_hist[2] - self.error_hist[1]
        dd1 = d1 - d2

        # 2ステップ先を予測（t+2 予測: y + 2Δy + 2Δ²y）
        error_predict = self.error_hist[3] + 2 * d1 + 2 * dd1

        inliers = np.array(best_inliers)
        x = inliers[:, 0]
        y = inliers[:, 1]
        A = np.vstack([x, np.ones(len(x))]).T
        m, _ = np.linalg.lstsq(A, y, rcond=None)[0]
        wall_angle = math.atan(m)

        # 平滑化
        alpha = 0.3
        wall_angle = alpha * wall_angle + (1 - alpha) * self.prev_wall_angle
        d_wall_angle = wall_angle - self.prev_wall_angle
        self.prev_wall_angle = wall_angle

        # Lidar情報保持
        self.wall_angle = wall_angle
        self.laser_ranges = ranges
        self.lidar_angle_min = angle_min
        self.lidar_angle_increment = angle_inc

        twist = Twist()

        # === コーナー検出 ===
        if self.mode == "FOLLOWING":
            distance_along_wall = float('inf')  # ← どの条件にも入らなかった場合用
            wall_dir_index = int(self.wall_angle / self.lidar_angle_increment)
            if 0 <= wall_dir_index :
                distance_along_wall = self.laser_ranges[wall_dir_index]

            elif wall_dir_index < 0 :
                distance_along_wall = self.laser_ranges[1800 + wall_dir_index]

            #self.get_logger().info(f" wall_angle={math.degrees(self.wall_angle):.1f}°, c_distance={distance_along_wall:.2f} m ")
            #self.get_logger().info(f" wall_dir_index={ wall_dir_index:.1f},len(self.laser_ranges)={len(self.laser_ranges):.1f}")
            if distance_along_wall <= 1.2  and self.segment == 1 : ######ここが，最初のカーブをする距離を決めるところ．
                #self.get_logger().info(f"[CORNER] wall_angle={math.degrees(self.wall_angle):.1f}°, c_distance={distance_along_wall:.2f} m → Turning!")
                self.mode = "TURNING"
                self.turn_start_time = self.get_clock().now()

            if distance_along_wall <= 1.15  and self.segment == 2 :
                #self.get_logger().info(f"[CORNER] wall_angle={math.degrees(self.wall_angle):.1f}°, c_distance={distance_along_wall:.2f} m → Turning!")
                self.mode = "TURNING"
                self.turn_start_time = self.get_clock().now()

            if distance_along_wall <= 1.18  and self.segment == 3 :
                #self.get_logger().info(f"[CORNER] wall_angle={math.degrees(self.wall_angle):.1f}°, c_distance={distance_along_wall:.2f} m → Turning!")
                self.mode = "TURNING"
                self.turn_start_time = self.get_clock().now()

            if distance_along_wall <= 1.23  and self.segment == 4  :
                #self.get_logger().info(f"[CORNER] wall_angle={math.degrees(self.wall_angle):.1f}°, c_distance={distance_along_wall:.2f} m → Turning!")
                self.mode = "TURNING"
                self.turn_start_time = self.get_clock().now()

        # === TURNING ===
        if self.mode == "TURNING":
            if self.laser_ranges is None or len(self.laser_ranges) == 0:
                self.get_logger().warn("Laser data not ready.")
                return

            front_idx = 0
            front_distance = self.laser_ranges[front_idx]

            now = self.get_clock().now()
            elapsed_turn = (now - self.turn_start_time).nanoseconds * 1e-9

            if elapsed_turn < 0.1:
                twist.linear.x = 0.0
                twist.angular.z = 0.35
                self.get_logger().info("Turning !!!")

            else:
                if front_distance > 1.4:
                    self.start_time = now
                    self.mode = "FOLLOWING"
                    self.segment += 1
                    if self.segment > 4:
                        self.segment = 1
                        self.turn_pub.publish(String(data='turn_end'))
                    #self.get_logger().info(f"[TURN END] segment → {self.segment}")

                else:
                    self.turn_start_time = now  # 継続ターン
                    twist.linear.x = 0.1
                    twist.angular.z = 0.25
                    #self.get_logger().info("Still blocked. Keep turning...")

        # === FOLLOWING ===
        if self.mode == "FOLLOWING":
                now = self.get_clock().now()

                if self.start_time is None:
                    self.start_time = now

                elapsed = (now - self.start_time).nanoseconds * 1e-9

                if elapsed < 3.0:
                    if self.segment==1:
                        twist.linear.x = 0.0
                        twist.angular.z = 0.0
                    else:
                        twist.linear.x = 0.15
                        twist.angular.z = - self.Kp * error_predict \
                                + self.angle_gain * wall_angle \
                                + self.wall_angle_d_gain * d_wall_angle

                elif elapsed < 4.0:
                        twist.linear.x = 0.20
                        twist.angular.z = - self.Kp * error_predict \
                                + self.angle_gain * wall_angle \
                                + self.wall_angle_d_gain * d_wall_angle

                else:
                        if self.segment == 1:
                                self.Kp = 0.33
                                self.Kd = 0.14
                                self.angle_gain = 0.32
                                self.wall_angle_d_gain = 0.35
                                twist.linear.x = 0.25
                                #self.get_logger().info('Segment1 following!!!')
                        else:
                                self.Kp = 0.25
                                self.Kd = 0.12
                                self.angle_gain = 0.25
                                self.wall_angle_d_gain = 0.25
                                twist.linear.x = 0.18
                                #self.get_logger().info('Segment2~4 following!!!')

                        twist.angular.z = - self.Kp * error_predict \
                                + self.angle_gain * wall_angle \
                                + self.wall_angle_d_gain * d_wall_angle

        # === ログ & 実行 ===
        front_idx = 0
        front_distance = self.laser_ranges[front_idx]

        #self.get_logger().info(
        #    f"wall_angle={math.degrees(wall_angle):.1f}°, -Kp*e={-self.Kp*error:.3f}, "
        #    f"angle_gain*wall={self.angle_gain * wall_angle:.3f}, wall_angle_d={d_wall_angle:.3f},predict={error_predict:.3f}, "
        #    f"distance_to_wall={distance:.3f}, front={front_distance:.3f}, linear.x={twist.linear.x:.2f}, angular.z={twist.angular.z:.2f}"
        #)

        self.cmd_pub.publish(twist)
# ──────────────────────────────────────────────
# 2) /change_mode で ACTIVE ⇄ INACTIVE をトグル
#    （名前はご要望どおり適当に…）
# ──────────────────────────────────────────────
class change_mode_listener(BasicNode):
    def __init__(self, target='wall_follower_lifecycle_node'):
        super().__init__('change_mode_listener')
        self.io_group = ReentrantCallbackGroup()
        self._cli       = self.create_client(ChangeState, f'{target}/change_state', callback_group=self.io_group)
        self._state_cli = self.create_client(GetState,   f'{target}/get_state', callback_group=self.io_group)
        self.create_subscription(String, 'mode_change', self._toggle_cb, 10, callback_group=self.io_group)

        # 両サービスが揃うのを待つ
        self._wait_for_service()

        # CONFIGURE → 成功後に ACTIVATE を送る
        self._send_transition(Transition.TRANSITION_CONFIGURE)
                              
        # 0.5 秒後に ACTIVATE
        self._once_timer = self.create_timer(0.2,self._activate_once) 

    def _activate_once(self):
        self.get_logger().info('first activate')
        self._send_transition(Transition.TRANSITION_ACTIVATE)
        self._once_timer.cancel()                  
    # ------------------------------------------------------------
    def _wait_for_service(self):
        self.get_logger().info('Waiting for lifecycle services…')
        while (not self._cli.wait_for_service(1.0) or
               not self._state_cli.wait_for_service(1.0)):
            self.get_logger().info('  …still waiting')
        self.get_logger().info('Service ready!')

    # ------------------------------------------------------------
    def _toggle_cb(self, _msg):
        # 現状態を問い合わせ
        req = GetState.Request()
        fut = self._state_cli.call_async(req)
        self.get_logger().info('toggle')
        fut.add_done_callback(self._state_done)
        #self.get_logger().info(f'{fut.result().current_state.id}')
        
        

    def _state_done(self, fut):
        self.get_logger().info('state_done')
        state_id = fut.result().current_state.id
        if state_id == State.PRIMARY_STATE_ACTIVE: # ACTIVE なら INACTIVE に
            self._send_transition(Transition.TRANSITION_DEACTIVATE)
        elif state_id == State.PRIMARY_STATE_INACTIVE:  # INACTIVE なら ACTIVE に
            self._send_transition(Transition.TRANSITION_ACTIVATE)

    def _send_transition(self, tid):
        self.get_logger().info(f'[DBG] _send_transition({tid}) called')
        req = ChangeState.Request()
        req.transition.id = tid
        fut = self._cli.call_async(req)
        self.get_logger().info(f'[DBG] future created: {fut}')
        fut.add_done_callback(partial(self._transition_done, tid))

    def _transition_done(self, tid, fut):
        ok = bool(fut.result() and fut.result().success)
        self.get_logger().info(f'Transition {tid} {"✓" if ok else "✗"}')


# ──────────────────────────────────────────────
#  main
# ──────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)

    positioning_node = wallfollwerModeNode()     # Lifecycle ノード
    trigger_node     = change_mode_listener()    # トグルノード

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(positioning_node)
    executor.add_node(trigger_node)

    try:
        executor.spin()
    finally:
        executor.shutdown()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
