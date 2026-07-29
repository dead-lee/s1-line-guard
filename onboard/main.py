# -*- coding: utf-8 -*-
"""
S1 Line Guard — 车载入口（占位，未实现）

目标状态机：
  INIT → PATROL ⇄ SCAN → LOCK → FIRE → RECOVER → PATROL

详见仓库根目录 README.md。
实现时在 RoboMaster App 实验室 Python 中落地；
此处仅保留结构说明，避免误当作可运行脚本直接拷贝开火。
"""

# 状态名（实现时使用）
STATE_INIT = "INIT"
STATE_PATROL = "PATROL"
STATE_SCAN = "SCAN"
STATE_LOCK = "LOCK"
STATE_FIRE = "FIRE"
STATE_RECOVER = "RECOVER"


def start():
    """
    App 实验室常见入口。

    TODO(M1+):
    - 初始化识别与云台姿态
    - 运行状态机主循环 / 事件驱动
    - PATROL / SCAN / LOCK / FIRE / RECOVER
    """
    # 故意不实现控制逻辑：当前 milestone = M0（需求与骨架）
    pass
