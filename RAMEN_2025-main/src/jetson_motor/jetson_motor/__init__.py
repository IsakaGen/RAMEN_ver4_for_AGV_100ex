import numpy as np
# 左モーター(M3,M4)
IN1 = 37
IN2 = 35
ENA = 32  # PWM1

# 右モーター(M1,M2)
IN3 = 36
IN4 = 38
ENB = 33

# エンコーダ入力ピン
ENC_A = 22
ENC_B = 21

# ロボットの物理パラメータ（定数として定義）
WHEEL_RADIUS = 0.065     # [m] タイヤ半径（6.5cm）
WHEEL_BASE = 0.285        # [m] 車輪間距離（30cm）
MAX_RPM = 144            # [RPM]
MAX_SPEED = (MAX_RPM/60)*WHEEL_RADIUS*2*3.14  # [m/s]
