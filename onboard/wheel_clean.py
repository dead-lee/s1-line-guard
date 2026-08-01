# WHEEL_CLEAN_VERSION=1.0.0 stamp=2026-08-01 13:48:46  (paste whole file; check stamp)
# -*- coding: utf-8 -*-
# S1 麦轮清洁程序 — 单文件，粘贴到 App 实验室 Python
#
# 作用：4 个全向轮持续转动约 60 秒，便于用刷子清洁；
#       轮速组合使底盘合力≈0，车身尽量原地不动。
#
# 挂到「自定义技能 / 自主程序」后可用 App 图标或车身按键触发（见文末说明）。
#
# 安全：
# - 车放平地、周围无障碍；轮子悬空更安全（架起底盘）
# - 手指远离轮缝；勿伸进麦轮滚子
# - 中途要停：App 停止程序 / 关电池

# =============================================================================
# CONFIG
# =============================================================================
CLEAN_TOTAL_S = 60.0      # 总清洁时长（秒）
PATTERN_S = 10.0          # 每种轮速组合保持时间（秒）
WHEEL_RPM = 250           # 轮速绝对值，范围约 [-1000, 1000]；太小刷不动，太大易抖
LOOP_PRINT_S = 5.0        # 控制台进度打印间隔

# =============================================================================
# 麦轮「抵消」组合 (lf, rf, lr, rr)
# 正=该轮前进方向旋转。对角/对侧反向可使 vx/vy/ω 接近 0，轮子仍在转。
# 多组轮换，让滚子不同方向都蹭到。
# =============================================================================
def make_patterns(s):
    return [
        (s, -s, -s, s),
        (-s, s, s, -s),
        (s, -s, s, -s),
        (-s, s, -s, s),
        (s, s, -s, -s),
        (-s, -s, s, s),
    ]

# =============================================================================
# helpers
# =============================================================================
def now_s():
    return tools.run_time_of_program()

def log(msg):
    print("[WHEEL_CLEAN t=%.1f] %s" % (now_s(), msg))

def leds_clean_mode():
    # 黄灯提示正在清洁
    led_ctrl.set_bottom_led(rm_define.armor_bottom_all, 255, 180, 0, rm_define.effect_always_on)
    led_ctrl.set_top_led(rm_define.armor_top_all, 255, 180, 0, rm_define.effect_always_on)

def leds_done():
    led_ctrl.set_bottom_led(rm_define.armor_bottom_all, 0, 255, 0, rm_define.effect_always_on)
    led_ctrl.set_top_led(rm_define.armor_top_all, 0, 255, 0, rm_define.effect_always_on)
    time.sleep(0.5)
    led_ctrl.turn_off(rm_define.armor_all)

def wheels_stop():
    chassis_ctrl.set_wheel_speed(0, 0, 0, 0)
    chassis_ctrl.stop()

def wheels_set(lf, rf, lr, rr):
    chassis_ctrl.set_wheel_speed(lf, rf, lr, rr)

# =============================================================================
# main
# =============================================================================
def start():
    print("======== Wheel Clean start ========")
    print("# WHEEL_CLEAN_VERSION=1.0.0 stamp=2026-08-01 13:48:46")
    log("begin total=%.0fs rpm=%d" % (CLEAN_TOTAL_S, WHEEL_RPM))

    robot_ctrl.set_mode(rm_define.robot_mode_free)
    wheels_stop()
    leds_clean_mode()

    patterns = make_patterns(WHEEL_RPM)
    n_pat = len(patterns)
    t0 = now_s()
    last_print = t0
    idx = 0
    pattern_t0 = t0

    # 先应用第一组
    lf, rf, lr, rr = patterns[0]
    wheels_set(lf, rf, lr, rr)
    log("pattern 0/ %d  lf=%d rf=%d lr=%d rr=%d" % (n_pat, lf, rf, lr, rr))

    while True:
        t = now_s()
        elapsed = t - t0
        if elapsed >= CLEAN_TOTAL_S:
            break

        # 轮换轮速组合
        if (t - pattern_t0) >= PATTERN_S:
            pattern_t0 = t
            idx = idx + 1
            if idx >= n_pat:
                idx = 0
            lf, rf, lr, rr = patterns[idx]
            wheels_set(lf, rf, lr, rr)
            log("pattern %d  lf=%d rf=%d lr=%d rr=%d" % (idx, lf, rf, lr, rr))

        if (t - last_print) >= LOOP_PRINT_S:
            last_print = t
            left = CLEAN_TOTAL_S - elapsed
            log("running... left=%.0fs pattern=%d" % (left, idx))

        time.sleep(0.1)

    wheels_stop()
    leds_done()
    log("done, wheels stopped")
    print("======== Wheel Clean done ========")

# =============================================================================
# 如何做成「按按钮触发」的自定义程序
# =============================================================================
# 1) 实验室 → 新建 Python → 粘贴本文件 → 保存（起名如 wheel_clean）
# 2) 在工程列表中，将该程序设为：
#    - 「自定义技能」：连接 S1 后进 FPV/单机驾驶，点对应技能图标运行
#    - 或「自主程序」：用智能中控侧面「自主程序」物理键一键运行（以你 App 界面文案为准）
# 3) 运行中要停止：App 点停止，或结束 60s 自动停
# 4) 若车仍缓慢挪动：把 WHEEL_RPM 略降，或微调 patterns 符号；地面打滑也会微移，建议架起底盘清洁
