# WHEEL_CLEAN_VERSION=1.2.0 stamp=2026-08-01 13:54:22  (paste whole file; check stamp)
# -*- coding: utf-8 -*-
# S1 麦轮清洁 v1.2 — 慢速原地转圈（清洁用）
#
# 抵消组合在实车上仍会后退，改默认：慢速原地自转。
# 麦轮 (lf,rf,lr,rr) ≈ (s,-s,s,-s) 主要为 omega，车绕自身转、侧移小。
# 转速保持很低，方便刷洗。
#
# 架起底盘更安全；中途 App 停止即可。

# =============================================================================
CLEAN_TOTAL_S = 60.0
PATTERN_S = 20.0
WHEEL_RPM = 50
LOOP_PRINT_S = 5.0

# =============================================================================
def now_s():
    return tools.run_time_of_program()

def log(msg):
    print("[WHEEL_CLEAN t=%.1f] %s" % (now_s(), msg))

def make_patterns(s):
    # 原地慢转：正转 / 反转 交替，四轮都在动
    return [
        (s, -s, s, -s),
        (-s, s, -s, s),
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
    print("# WHEEL_CLEAN_VERSION=1.2.0 stamp=2026-08-01 13:54:22")
    log("slow spin clean rpm=%d total=%.0fs" % (WHEEL_RPM, CLEAN_TOTAL_S))

    robot_ctrl.set_mode(rm_define.robot_mode_free)
    wheels_stop()
    time.sleep(0.2)
    leds_clean_mode()

    s = WHEEL_RPM
    if s < 0:
        s = -s
    if s > 120:
        s = 120
    if s < 20:
        s = 20

    patterns = make_patterns(s)
    n_pat = len(patterns)
    t0 = now_s()
    last_print = t0
    idx = 0
    pattern_t0 = t0

    lf, rf, lr, rr = patterns[0]
    wheels_set(lf, rf, lr, rr)
    log("spin pattern %d lf=%d rf=%d lr=%d rr=%d" % (idx, lf, rf, lr, rr))

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
            log("spin pattern %d lf=%d rf=%d lr=%d rr=%d" % (idx, lf, rf, lr, rr))

        if (t - last_print) >= LOOP_PRINT_S:
            last_print = t
            log("left=%.0fs dir=%d" % (CLEAN_TOTAL_S - elapsed, idx))

        time.sleep(0.15)

    wheels_stop()
    leds_done()
    log("done stopped")
    print("======== Wheel Clean done ========")
