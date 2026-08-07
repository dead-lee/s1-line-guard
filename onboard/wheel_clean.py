# WHEEL_CLEAN_VERSION=1.2.1 stamp=2026-08-01 20:53:07  (paste whole file; check stamp)
# -*- coding: utf-8 -*-
# S1 麦轮清洁 v1.2.1 — 慢速原地转圈（清洁用，单文件粘贴进 App 实验室）
#
# 背景：
#   麦轮「四轮反向抵消」在实车上仍易前后漂移，故采用慢速原地自转：
#   set_wheel_speed(lf, rf, lr, rr) = (s, -s, s, -s) 或反向，车绕竖直轴慢转，
#   四轮持续转动，方便用刷子清洁。
#
# 使用：
#   1. 实验室新建 Python，全选粘贴本文件，看首行 stamp
#   2. 可设为「自定义技能 / 自主程序」，用 App 图标或中控按键触发
#   3. 建议架起底盘；手指远离轮缝；中途用 App 停止
#
# 参数：见下方 CONFIG（总时长 / 转速 / 换向间隔）

# =============================================================================
# CONFIG
# =============================================================================
CLEAN_TOTAL_S = 60.0   # 清洁总时长（秒）
PATTERN_S = 20.0       # 每组轮速保持多久后换向（秒）
WHEEL_RPM = 50         # 轮速绝对值，慢速；可改 30~80
LOOP_PRINT_S = 5.0     # 控制台进度打印间隔（秒）

# =============================================================================
# 工具函数
# =============================================================================
def now_s():
    """程序已运行秒数（App 实验室时钟）。"""
    return tools.run_time_of_program()

def log(msg):
    """带时间戳打印到实验室控制台。"""
    print("[WHEEL_CLEAN t=%.1f] %s" % (now_s(), msg))

def make_patterns(s):
    """
    生成原地慢转的轮速组合列表。
    参数 s: 转速绝对值 (rpm)。
    返回: [(lf, rf, lr, rr), ...]
      - 第一组：左前/左后 正转，右前/右后 反转 → 车向一侧自转
      - 第二组：全部取反 → 反向自转
    两组轮换，滚子正反方向都能刷到。
    """
    return [
        (s, -s, s, -s),
        (-s, s, -s, s),
    ]

def leds_clean_mode():
    """黄灯：提示正在清洁。"""
    led_ctrl.set_bottom_led(rm_define.armor_bottom_all, 255, 180, 0, rm_define.effect_always_on)
    led_ctrl.set_top_led(rm_define.armor_top_all, 255, 180, 0, rm_define.effect_always_on)

def leds_done():
    """结束时绿灯闪一下再熄灭。"""
    led_ctrl.set_bottom_led(rm_define.armor_bottom_all, 0, 255, 0, rm_define.effect_always_on)
    led_ctrl.set_top_led(rm_define.armor_top_all, 0, 255, 0, rm_define.effect_always_on)
    time.sleep(0.4)
    led_ctrl.turn_off(rm_define.armor_all)

def wheels_stop():
    """四轮停转。"""
    chassis_ctrl.set_wheel_speed(0, 0, 0, 0)
    chassis_ctrl.stop()

def wheels_set(lf, rf, lr, rr):
    """
    直接设定四轮转速 (rpm)。
    lf/rf/lr/rr = 左前 / 右前 / 左后 / 右后；正负表示转向。
    """
    chassis_ctrl.set_wheel_speed(lf, rf, lr, rr)

# =============================================================================
# 入口：App 实验室 start()
# =============================================================================
def start():
    """
    主流程（按时间顺序）：
      1) 打印版本，方便确认粘贴的是最新代码
      2) 自由模式 + 先停轮，避免残留速度
      3) 黄灯进入清洁态
      4) 钳制转速到安全慢速区间，生成正转/反转两组图案
      5) 下发第一组轮速，进入主循环
      6) 主循环直到 CLEAN_TOTAL_S：
           - 到点换下一组轮速（正转 <-> 反转）
           - 定期打日志（剩余时间）
      7) 停轮、绿灯提示结束
    """
    # --- 1) 版本与开始日志 ---
    print("======== Wheel Clean start ========")
    print("# WHEEL_CLEAN_VERSION=1.2.1 stamp=2026-08-01 20:53:07")
    log("slow spin clean rpm=%d total=%.0fs" % (WHEEL_RPM, CLEAN_TOTAL_S))

    # --- 2) 底盘自由模式；清零轮速，避免启动时猛冲 ---
    robot_ctrl.set_mode(rm_define.robot_mode_free)
    wheels_stop()
    time.sleep(0.2)

    # --- 3) 灯效：黄色 = 清洁进行中 ---
    leds_clean_mode()

    # --- 4) 转速安全钳制（过快难刷、过慢可能几乎不转）---
    s = WHEEL_RPM
    if s < 0:
        s = -s
    if s > 120:
        s = 120
    if s < 20:
        s = 20

    # 两组自转图案：正转 / 反转
    patterns = make_patterns(s)
    n_pat = len(patterns)

    # 计时基准
    t0 = now_s()           # 程序段开始时刻
    last_print = t0        # 上次进度日志时刻
    idx = 0                # 当前图案下标
    pattern_t0 = t0        # 当前图案开始时刻

    # --- 5) 立即应用第一组轮速，开始原地慢转 ---
    lf, rf, lr, rr = patterns[0]
    wheels_set(lf, rf, lr, rr)
    log("spin pattern %d lf=%d rf=%d lr=%d rr=%d" % (idx, lf, rf, lr, rr))

    # --- 6) 主循环：按时长结束；中间换向 + 打进度 ---
    while True:
        t = now_s()
        elapsed = t - t0

        # 总时长到：跳出，进入收尾
        if elapsed >= CLEAN_TOTAL_S:
            break

        # 当前图案已保持 PATTERN_S 秒 → 换下一组（正转/反转轮换）
        if (t - pattern_t0) >= PATTERN_S:
            pattern_t0 = t
            idx = idx + 1
            if idx >= n_pat:
                idx = 0
            lf, rf, lr, rr = patterns[idx]
            wheels_set(lf, rf, lr, rr)
            log("spin pattern %d lf=%d rf=%d lr=%d rr=%d" % (idx, lf, rf, lr, rr))

        # 每隔 LOOP_PRINT_S 秒打印剩余时间，方便控制台确认仍在跑
        if (t - last_print) >= LOOP_PRINT_S:
            last_print = t
            log("left=%.0fs dir=%d" % (CLEAN_TOTAL_S - elapsed, idx))

        # 稍睡，避免空转占满 CPU（实验室循环惯例）
        time.sleep(0.15)

    # --- 7) 收尾：停轮 + 绿灯提示完成 ---
    wheels_stop()
    leds_done()
    log("done stopped")
    print("======== Wheel Clean done ========")
