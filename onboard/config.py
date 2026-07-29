# -*- coding: utf-8 -*-
"""
S1 Line Guard — 可调参数（占位）

注意：
- 本文件目前仅为规格占位，尚未接入真实车载控制逻辑。
- 粘贴到 App 时，需改写成实验室环境可识别的写法（可能无需独立模块文件）。
"""

# 巡线
LINE_COLOR = "blue"  # 仅红/绿/蓝；本项目固定蓝

# 分时巡逻 / 扫描（秒）
T_MOVE = 2.0
T_SCAN = 4.0

# 云台（角度单位以实现时 App API 为准，以下为设计值）
YAW_SCAN_MIN = -70
YAW_SCAN_MAX = 70
PITCH_LINE = -20   # 巡线俯视（实车标定）
PITCH_SCAN = 0     # 扫人平视（实车标定）

# 告警 / 开火
T_WARN_BEFORE_FIRE = 3.0  # 发现人后首次点射前等待
T_FIRE_INTERVAL = 1.0     # 点射间隔
T_CLEAR = 2.0             # 连续无人多久算离开

# 安全：联调时可先 False，只做灯效与锁定
ENABLE_FIRE = False
