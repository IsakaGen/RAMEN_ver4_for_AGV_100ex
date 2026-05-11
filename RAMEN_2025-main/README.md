# 概要
- 麺類自動調理システムver.4のAGVに搭載されてるプログラムです．
# 使い方
- 以下の説明は全てAGV1の前提で書かれています．AGV2を動かす場合はAGV1とAGV2を入れ替えて読んでください．
## 一台体制
1. AGV1をスタート位置に置く
2. ターミナルにて以下のコマンドを打つ
```
ros2 launch status_manager AGV_launch
```
3. ターミナルをもう一つ起動し，以下のコマンドを打つとAGVが移動を開始する
```
ros2 topic pub /AGV2_Button_pushed std_msgs/String "data:" -1
```
## 二台体制
1. AGV1をスタート地点，AGV2を提供位置に置く．
2. それぞれのターミナルで以下のコマンドを打つ
```
ros2 launch status_manager AGV_launch
```
3. AGV2でのみターミナルをもう一つ起動し，以下のコマンドを打つ
```
ros2 topic pub /force_status_set std_msgs/Int16 "data: 4"
```
4. AGV2のボタンを押すと二台とも移動を開始する．
# 仕様説明
- ros2の知識があることを前提とした説明です．
- 全てAGV1の前提で書かれています．AGV2の場合はAGV1とAGV2を入れ替えて読んでください．
## 使用ノード一覧
- manager
  - パッケージ：status_manager
  - AGVの内部状態を管理するノード．この内部状態（status）の番号によってAGVが反応すべきトピックとそうでないトピックを判別している．（詳しくは後述）
- wall_follower_lifecycle
  - パッケージ：transition
  - lidarから得られる点群データから右の壁との距離を計算し，そこから左右のモーターが出すべき速度を算出，publishするノード．またlifecycleノードといい，Active状態とInactive状態を切り替えれる．
- sllidar_node
  - パッケージ：sllidar_ros2
  - 搭載しているlidarをros2で使うためのノード．公式が公開しているものをそのまま利用している．
- joy_listener
  - パッケージ：jetson_motor
  - 左右モーターの出すべき速度を受けて，エンコーダの入力からモーターに送るべき電圧（duty比）を計算し，モーターを動かすノード．停止，移動の切り替えもこのノードで行う．
  なおモーターの移動，停止状態もプログラムではstatusと呼称しているが，内部状態と混乱するため，ここではmotor statusと呼ぶ
- button_publisher
  - パッケージ：button
  - 提供完了時にボタンが押されたことを検知するノード．
- target_stop  
  - パッケージ：move_target
  - AGVが提供位置，スタート位置に着いた時に停止命令を出すノード．また今が提供位置を目指しているか，スタート位置を目指しているか，それ以外かを変数targetで管理している
## 内部でのシーケンス
1. 全てのノードが起動．
    - manager: statusが0に
    - wall_follower_lifecycle: Active状態，モーターの速度を/AGV1/cmd_velでpublish
    - sllidar_node: 以後ずっと点群を/AGV1/scanでpublish
    - joy_listener: motor statusが0(停止状態)に．/AGV1/cmd_velをsubscribeしているが，無視している状態．
    - target_stop: targetをNoneに
2. /AGV2_Button_pushedがpublishされ移動開始．
    1. managerが/AGV2_Button_pushedをsubscribe．statusを1に変更し，/startをpublishする．
    2. joy_listenerが/start (data: AGV1)をsubscribeし，motor statusを1(移動状態)に．/AGV1/cmd_velをもとにモーターを動かす．
3. UR Aのカメラにマーカーが映り，停止．
    1. カメラにマーカーが映り，/tag_detection_1がpublishされる．
    2. managerが/tag_detection_1をsubscribe．statusを2に変更し，/AGV1/mode_changeをpublish
    3. wall_follower_lifecycleが/AGV1/mode_changeをsubscribe．Inactive状態になり，/AGV1/cmd_velのpublishを停止．
      入れ替わりでカメラのノードからタグの位置をもとにした/AGV1/cmd_velがpublishされるようになる．（停車位置付近では制御則を切り替えている）
    4. カメラに映るタグの位置が目標位置に達したらカメラのノードが/stop (data: AGV1)をpublishする．
    5. joy_listenerが/stop (data: AGV1)をsubscribe．motor statusを0に．
4. 麺の盛り付けが完了し，移動再開．
    1. UR_Aのノードが/noodle_doneをpublish．
    2. managerが/noodle_doneをsubscribe．/AGV1/mode_changeと/start (data: AGV1)をpublish
    3. wall_follower_lifecycleが/AGV1/mode_changeをsubscribe．Activeになり，/AGV1/cmd_velのpublishを再開．
    4. 同時にjoy_listenerが/start (data: AGV1)をsubscribeし，motor statusを1(移動状態)に．/AGV1/cmd_velをもとにモーターを動かす．
5. UR B, Cのカメラにマーカーが映り，停止．
    1. カメラにマーカーが映り，/tag_detection_2がpublishされる．
    2. managerが/tag_detection_2をsubscribe．statusを3に変更し，/AGV1/mode_changeをpublish
    3. wall_follower_lifecycleが/AGV1/mode_changeをsubscribe．Inactive状態になり，/AGV1/cmd_velのpublishを停止．
      入れ替わりでカメラのノードからタグの位置をもとにした/AGV1/cmd_velがpublishされるようになる．（停車位置付近では制御則を切り替えている）
    4. カメラに映るタグの位置が目標位置に達したらカメラのノードが/stop (data: AGV1)をpublishする．
    5. joy_listenerが/stop (data: AGV1)をsubscribe．motor statusを0に．
6. トッピングの盛り付けが完了し，移動再開．
    1. UR_B. Cのノードが/negi_doneをpublish．
    2. managerが/negi_doneをsubscribe．/AGV1/mode_change, /AGV1/move_target (data: serve)，/start (data: AGV1)をpublish
    3. wall_follower_lifecycleが/AGV1/mode_changeをsubscribe．Activeになり，/AGV1/cmd_velのpublishを再開．
    4. 同時にjoy_listenerが/start (data: AGV1)をsubscribeし，motor statusを1(移動状態)に．/AGV1/cmd_velをもとにモーターを動かす．
    5. 同時にtarget_stopが/AGV1/move_target (data: serve)をsubscribeし，targetをserveに設定
7. 提供位置に到着し，停止．
    1. target_stopが/AGV1/scanから正面との距離を計測し，目標距離を下回ったら/AGV1/serve_arraiveと/stop (data: AGV1)をpublish
    2. managerが/AGV1/serve_arraiveをsubscribe．statusを4に変更する．
    3. 同時にjoy_listenerが/stop (data: AGV1)をsubscribe．motor statusを0に．
8. 提供完了後，ボタンが押され，移動再開
    1. ボタンが押され，button_publisherが/AGV1_Button_pushedをpublish
    2. managerが/AGV1_Button_pushedをsubscribe，statusを0にし，/AGV1/move_target (data: return)，/start (data: AGV1)をpublish
    3. joy_listenerが/start (data: AGV1)をsubscribeし，motor statusを1(移動状態)に．/AGV1/cmd_velをもとにモーターを動かす．
    4. 同時にtarget_stopが/AGV1/move_target (data: return)をsubscribeし，targetをreturnに設定
9. スタート位置に帰還し，停止．
    1. 帰還途中で，カーブを曲がり終わるたびに/AGV1/turningをpublish
    2. target_stopが/AGV1/turningをsubscribe．カーブを曲がった回数をカウントする．
    3. target_stopでカーブのカウントが4になったら，/stop (data: AGV1)をpublish．
    4. joy_listenerが/stop (data: AGV1)をsubscribe．motor statusを0に．
10. 2に戻る．
## その他
- AGVの名前（AGV1 or AGV2）はstatus_managerのinit.pyに設定しており，ここを書き換えれば自動的に全トピックの名前も変更されます．
- topicの中身は(data: )で書かれているものにする必要があります．逆に，説明中に(data: )が書かれていないtopicは中身がなんでも問題ないです．
- 二台のAGVが混線しないよう，managerのstatusが適切でないとsubscribeしたtopicを無視します．例えば/noodle_doneはstatusが2以外の時はsubscribeしても何も起きません
- 二台体制で使っている/AGV1/force_status_setはmanagerのstatusを書き換えるためのtopicです．上記の通りstatusが適切でないとうまく動かないので単体試験をするときなど途中から動かすときはこれでstatusを設定してください．
- name spaceを活用してAGV1とAGV2の混線を回避しています．また各AGVの内部で完結しないトピックは意図的にname spaceの適用外にしています．
- 説明で出てきていないパッケージやノードがまだいろいろあります．開発中に没になったものや，自律走行ができなかった時にジョイコン操作できるように用意したものなどいろいろあるので見ておくといいかもしれません．
