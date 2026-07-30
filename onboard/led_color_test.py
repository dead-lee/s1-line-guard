# -*- coding: utf-8 -*-
"""
S1 Line Guard — 彩灯颜色变换测试（单文件，可整段粘贴进 App 实验室）

用途：验证 App 实验室 Python 能运行，以及装甲灯 / 云台灯 API 是否正常。
操作：全选本文件内容 → 粘贴到 RoboMaster App → 实验室 → Python → 运行。

注意：
- 仅使用车载官方 API，无第三方库。
- 若某条 API 报错，把报错信息记下（不同固件/App 枚举名偶有差异）。
"""

# =============================================================================
# 可调参数
# =============================================================================

# 每种颜色保持时间（秒）
HOLD_SEC = 1.5

# 颜色列表：(名称, R, G, B)  取值 0~255
COLOR_SEQ = [
    ("红", 255, 0, 0),
    ("橙", 255, 80, 0),
    ("黄", 255, 200, 0),
    ("绿", 0, 255, 0),
    ("青", 0, 200, 255),
    ("蓝", 0, 80, 255),
    ("紫", 160, 0, 255),
    ("白", 255, 255, 255),
]


# =============================================================================
# 灯效工具
# =============================================================================

def leds_solid(r, g, b):
    """底盘装甲灯 + 云台灯 常亮同色。"""
    # 底盘四周
    led_ctrl.set_bottom_led(
        rm_define.armor_bottom_all, r, g, b, rm_define.effect_always_on
    )
    # 云台两侧
    led_ctrl.set_top_led(
        rm_define.armor_top_all, r, g, b, rm_define.effect_always_on
    )


def leds_off():
    """关闭全部灯。"""
    led_ctrl.turn_off(rm_define.armor_all)


def leds_breath(r, g, b):
    """呼吸灯效（若固件不支持 effect_breath，运行到此可能报错，可注释掉 start 里对应段）。"""
    led_ctrl.set_bottom_led(
        rm_define.armor_bottom_all, r, g, b, rm_define.effect_breath
    )
    led_ctrl.set_top_led(
        rm_define.armor_top_all, r, g, b, rm_define.effect_breath
    )


# =============================================================================
# 主流程
# =============================================================================

def start():
    """
    App 实验室入口：依次变换颜色 → 红色呼吸几秒 → 关灯结束。
    """
    # --- 1) 彩虹轮换：每种颜色常亮 HOLD_SEC 秒 ---
    i = 0
    while i < len(COLOR_SEQ):
        name, r, g, b = COLOR_SEQ[i]
        leds_solid(r, g, b)
        # 名称仅作阅读注释；车端无 print 到 Mac 控制台的保证
        time.sleep(HOLD_SEC)
        i = i + 1

    # --- 2) 红色呼吸（演示另一种灯效）---
    leds_breath(255, 0, 0)
    time.sleep(3)

    # --- 3) 快速红闪提示结束（用 flash 频率接口）---
    leds_solid(255, 0, 0)
    led_ctrl.set_flash(rm_define.armor_all, 4)  # 约 4 Hz，范围常见 1~10
    time.sleep(2)

    # --- 4) 关灯 ---
    leds_off()
