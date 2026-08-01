# WHEEL_CLEAN_VERSION=1.1.0 stamp=2026-08-01 13:52:23  (paste whole file; check stamp)
# -*- coding: utf-8 -*-
# S1 麦轮清洁程序 v1.1 — 慢速、尽量原地不动
#
# 麦克纳姆运动学（轮速 lf,rf,lr,rr）：
#   vx ~ (lf+rf+lr+rr)/4
#   vy ~ (-lf+rf+lr-rr)/4
#   w  ~ (-lf+rf-lr+rr)/4
# 要 vx=vy=w=0，一组解是：lf=rf=s, lr=rr=-s  即 (s, s, -s, -s)
# 以及反向 (-s, -s, s, s)。四轮都在转，合力/合力矩约为 0。
#
# 上一版用了 (s,-s,-s,s) 等组合，在麦轮上会产生侧移，已改掉。
# CLEAN_MODE:
#   "hold"  — 仅用抵消组合，车身尽量不动（推荐清洁）
#   "spin"  — 慢速原地自转 (s,-s,s,-s)，若 hold 仍微移可改用这个
#
# 挂自定义技能 / 自主程序后按键触发。架起底盘清洁更安全。

# =============================================================================
# CONFIG — 清洁用慢速
# =============================================================================
CLEAN_TOTAL_S = 60.0
PATTERN_S = 15.0
WHEEL_RPM = 60            # 慢！清洁不是冲锋；可试 40~100
CLEAN_MODE = "hold"       # "hold" 原地抵消 | "spin" 慢速自转
LOOP_PRINT_S = 5.0

# =============================================================================
def now_s():
    return tools.run_time_of_program()

def log(msg):
    print("[WHEEL_CLEAN t=%.1f] %s" % (now_s(), msg))

def make_patterns(s):
    if CLEAN_MODE == "spin":
        # 纯自转：vx=vy=0, w≠0；慢速原地转
        return [
            (s, -s, s, -s),
            (-s, s, -s, s),
        ]
    # hold：理论上 vx=vy=w=0，四轮仍转
    return [
        (s, s, -s, -s),
        (-s, -s, s, s),
    ]

def leds_clean_mode():
    led_ctrl.set_bottom_led(rm_define.armor_bottom_all, 255, 180, 0, rm_define.effect_always_on)
    led_ctrl.set_top_led(rm_define.armor_top_all, 255, 180, 0, rm_define.effect_always_on)

def leds_done():
    led_ctrl.set_bottom_led(rm_define.armor_bottom_all, 0, 255, 0, rm_define.effect_always_on)
    led_ctrl.set_top_led(rm_define.armor_top_all, 0, 255, 0, rm_define.effect_always_on)
    time.sleep(0.4)
    led_ctrl.turn_off(rm_define.armor_all)

def wheels_stop():
    chassis_ctrl.set_wheel_speed(0, 0, 0, 0)
    chassis_ctrl.stop()

def wheels_set(lf, rf, lr, rr):
    chassis_ctrl.set_wheel_speed(lf, rf, lr, rr)

def start():
    print("======== Wheel Clean start ========")
    print("# WHEEL_CLEAN_VERSION=1.1.0 stamp=2026-08-01 13:52:23")
    log("mode=%s rpm=%d total=%.0fs" % (CLEAN_MODE, WHEEL_RPM, CLEAN_TOTAL_S))

    robot_ctrl.set_mode(rm_define.robot_mode_free)
    wheels_stop()
    time.sleep(0.2)
    leds_clean_mode()

    s = WHEEL_RPM
    if s < 0:
        s = -s
    if s > 200:
        s = 200
    patterns = make_patterns(s)
    n_pat = len(patterns)
    t0 = now_s()
    last_print = t0
    idx = 0
    pattern_t0 = t0

    lf, rf, lr, rr = patterns[0]
    wheels_set(lf, rf, lr, rr)
    log("pattern %d lf=%d rf=%d lr=%d rr=%d" % (idx, lf, rf, lr, rr))

    while True:
        t = now_s()
        elapsed = t - t0
        if elapsed >= CLEAN_TOTAL_S:
            break

        if (t - pattern_t0) >= PATTERN_S:
            pattern_t0 = t
            idx = idx + 1
            if idx >= n_pat:
                idx = 0
            lf, rf, lr, rr = patterns[idx]
            wheels_set(lf, rf, lr, rr)
            log("pattern %d lf=%d rf=%d lr=%d rr=%d" % (idx, lf, rf, lr, rr))

        if (t - last_print) >= LOOP_PRINT_S:
            last_print = t
            log("left=%.0fs pattern=%d" % (CLEAN_TOTAL_S - elapsed, idx))

        time.sleep(0.15)

    wheels_stop()
    leds_done()
    log("done stopped")
    print("======== Wheel Clean done ========")

# 自定义技能 / 自主程序：保存后在列表里设为技能或自主，FPV 图标或中控按键触发。
# 若 hold 仍侧移：改 CLEAN_MODE = "spin"，或 WHEEL_RPM = 40，并架起底盘。
