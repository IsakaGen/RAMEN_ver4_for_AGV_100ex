import rclpy
from rclpy.lifecycle import Node                      # Lifecycle 基底クラス
from rclpy.lifecycle import TransitionCallbackReturn
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import ChangeState, GetState
from lifecycle_msgs.msg import Transition
from rclpy.node import Node as BasicNode              # Lifecycle ではない通常の Node
from std_msgs.msg import String
from functools import partial                         # 非同期 callback で使う


# ──────────────────────────────────────────────
# 1) 位置決めモード用 Lifecycle ノード
# ──────────────────────────────────────────────
class PositioningModeNode(Node):
    def __init__(self):
        super().__init__('positioning_mode_node')
        self._once_timer = None                       # active 専用タイマー（メインの関数を呼び出すのに使う）

    # --- Lifecycle callbacks --------------------------------------------------
    def on_configure(self, state: State) -> TransitionCallbackReturn:  #変数TransitionCallbackReturnを受け取っている．
        self.get_logger().info('Configuring…')                         #同じ変数を受け取っているように見える．しかし，ライブラリrclpy.lifecycleは有能なAPIで，TransitionCallbackReturnがUNCONFIGURED→INACTIVEとなったときだけ，この関数が呼び出されるように設定してくれている．
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:   # TransitionCallbackReturnがINACTIVE→ACTIVEとなったときに呼び出される．　
        self.get_logger().info('PositioningModeNode is ACTIVE')         
        self._once_timer = self.create_timer(
            0.2, self._announce_positioning_mode)     # ← タイマー開始
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State) -> TransitionCallbackReturn: # TransitionCallbackReturnがACTIVE→INACTIVEとなったときに呼び出される．
        self.get_logger().info('PositioningModeNode is DEACTIVATED')
        if self._once_timer:
            self._once_timer.cancel()                 # タイマー停止
        return TransitionCallbackReturn.SUCCESS

    # --- internal -------------------------------------------------------------
    def _announce_positioning_mode(self):             #タイマーによって定期的に呼び出される関数．ここに動作の内容を詰め込みます！いまは，メッセージを垂れ流すだけ
        self.get_logger().info('位置決めモードだあ！')
        # self._once_timer.cancel()  # 1 回だけこの関数を呼び出したい場合はコメントアウト削除


# ──────────────────────────────────────────────
# 2) ボタン信号 → Lifecycle 制御ノード
# ──────────────────────────────────────────────
class ButtonTriggerNode(BasicNode):
    def __init__(self, target='positioning_mode_node'):
        super().__init__('button_trigger_node')

        # --- service clients -------------------------------------------------
        self._cli       = self.create_client(ChangeState, f'/{target}/change_state')    # ChangeState 用 Client（状態遷移を要求）
        self._state_cli = self.create_client(GetState,   f'/{target}/get_state')        # GetState 用 Client（現在状態を取得）

        # --- button subscriber ----------------------------------------------
        self.subscription = self.create_subscription(
            String, '/Button_pushed', self._button_cb, 10)  # ボタンを押したときのトピック'/button_status'を受信し，関数_button_cbを呼び出す                             

        self._wait_for_service()

    # --- utility -------------------------------------------------------------
    def _wait_for_service(self):
        self.get_logger().info('Waiting for lifecycle service…')        
        while (not self._cli.wait_for_service(1.0) or                       #2 つのサービスがそろうまで 1 秒おきに待ち続ける
               not self._state_cli.wait_for_service(1.0)):
            self.get_logger().info('  …still waiting')
        self.get_logger().info('Service ready!')

    # --- callbacks -----------------------------------------------------------
    def _button_cb(self, msg: String):
        """ボタンを押すたびに現在状態を取得→次トランジションを非同期送信"""
        self.get_logger().info(f'[受信] {msg.data}')

        # ① /get_state を非同期呼び出し
        req = GetState.Request()                    # GetState.Request() は空のリクエスト
        fut = self._state_cli.call_async(req)       #self._state_cliは，ライフサイクル状態照会サービスのクライアント．Client.call_async(request) の戻り値は rclpy.task.Future と公式に明記されている。
        fut.add_done_callback(self._state_done)     # futに何か値が入ったら _state_done関数が呼び出されるように登録

    def _state_done(self, future):                  #さっき受け取ったfutureを受け取る．
        """/get_state 応答後：状態に応じて次のトランジションを非同期送信"""
        state_id = future.result().current_state.id #future.result() で応答メッセージを取り出し、その中の current_state.id（整数 1‥6）を取得。

        if state_id == State.PRIMARY_STATE_UNCONFIGURED:    #UNCONFIGURED (=1) のときはconfigureになってねていう命令
            self._send_transition(Transition.TRANSITION_CONFIGURE)

        elif state_id == State.PRIMARY_STATE_INACTIVE:      #INACTIVE (=2) のときはactivateになってねていう命令            
            self._send_transition(Transition.TRANSITION_ACTIVATE)

        elif state_id == State.PRIMARY_STATE_ACTIVE:        #ACTIVE (=3) のときはdeactivateになってねていう命令
            self._send_transition(Transition.TRANSITION_DEACTIVATE)

        else:
            self.get_logger().warning(f'想定外の状態 id={state_id}')

    # --- helpers -------------------------------------------------------------
    def _send_transition(self, transition_id):      #Transitoin IDを受け取っている
        """/change_state を非同期送信し、完了後にログを残す"""
        req = ChangeState.Request()              # ChangeState.Request() は空のリクエスト
        req.transition.id = transition_id       
        fut = self._cli.call_async(req)         #self._cliは，ライフサイクル状態遷移サービスのクライアント．Client.call_async(request) の戻り値は rclpy.task.Future と公式に明記されている。
        fut.add_done_callback(partial(self._transition_done, transition_id))        # futに何か値が入ったら _transition_done関数が呼び出されるように登録

    def _transition_done(self, transition_id, future):
        ok = bool(future.result() and future.result().success)  #サービス呼び出しが成功したかどうかを True / False で判定
        self.get_logger().info(f'Transition {transition_id} {"✓" if ok else "✗"}')


# ──────────────────────────────────────────────
#  main
# ──────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)                     # Ros2通信の初期化

    positioning_node = PositioningModeNode()    # Lifecycle ノードのインスタンス化
    trigger_node     = ButtonTriggerNode()      # ボタン信号 → Lifecycle 制御ノードのインスタンス化

    executor = rclpy.executors.MultiThreadedExecutor()  # Executor のインスタンス化（マルチスレッドで動作する）
    executor.add_node(positioning_node)         # Lifecycle ノードを Executor に登録
    executor.add_node(trigger_node)             # ボタン信号 → Lifecycle 制御ノードを Executor に登録

    try:
        executor.spin()                              # rclpy.shutdown() で抜ける
    finally:
        executor.shutdown()
        # shutdown() 済みなら destroy_node() は自動で安全にスキップされる
        rclpy.try_shutdown()                         # 念のため冪等呼び出し


if __name__ == '__main__':
    main()
