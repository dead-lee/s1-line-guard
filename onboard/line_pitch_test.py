# LINE_PITCH_TEST_VERSION=1.4.0 stamp=2026-08-04 13:55:00
# -*- coding: utf-8 -*-
# 线识别 + 俯仰扫描 + 完整返回值转储
#
# 用法：车压蓝线，整文件粘贴运行。控制台必须出现 VERSION 1.4.0。
# 目的：
#   1) 绝对俯仰从 0 到 -20，看「低头到多少仍能 SEE」
#   2) 把 get_line_detection_info 经 RmList 后的元素打全，形成对返回值的理解
#
# 已知（官方 + 你车上实测）：
#   - 无线时常 len≈2
#   - 有线时常 len==42，且 [2]>=1，[19] 为线 x（约 0~1，0.5=画面中）
#   - 必须 RmList(...)，与官方示例一致

# 绝对俯仰（度）：文档 pitch 约 [-20, 35]；越负越低头
PITCH_LIST = [5, 0, -5, -8, -10, -12, -14, -15, -16, -17, -18, -19, -20]
SAMPLES = 12
SAMPLE_DT = 0.05
SETTLE = 0.55
# 每档只完整 dump 一次返回值（避免刷屏）
DUMP_FULL_ONCE = True

def fetch_rmlist():
    raw = vision_ctrl.get_line_detection_info()
    try:
        return RmList(raw), "RmList"
    except Exception:
        return raw, "list"

def iget(info, i):
    try:
        return info[i]
    except Exception:
        return None

def ilen(info):
    try:
        return len(info)
    except Exception:
        return -1

def dump_full(info, kind, tag):
    """把整表打到日志，形成对函数返回的理解。"""
    n = ilen(info)
    print("[LPT] ---- FULL DUMP tag=%s kind=%s len=%d ----" % (tag, kind, n))
    # 逐个打印；RmList 可能是 1-based，也尝试 0..n
    # 先按 0..n-1（与 Python list 一致时）
    i = 0
    line = ""
    while i < n:
        v = iget(info, i)
        line = line + "[%d]=%s " % (i, str(v))
        if (i + 1) % 6 == 0:
            print("[LPT]   " + line)
            line = ""
        i = i + 1
    if line != "":
        print("[LPT]   " + line)
    # 官方关心的字段再强调一行
    print(
        "[LPT] KEY official: [2]=%s (pts?)  [19]=%s (cx?)  len==42? %s"
        % (str(iget(info, 2)), str(iget(info, 19)), str(n == 42))
    )
    print("[LPT] ---- END DUMP ----")

def official_see(info):
    """与官方一致：len==42 and [2]>=1 → 有线，cx=[19]。"""
    n = ilen(info)
    pts = 0
    cx = 0.5
    try:
        pts = int(iget(info, 2))
    except Exception:
        pts = 0
    try:
        if n > 19:
            cx = float(iget(info, 19))
    except Exception:
        cx = 0.5
    if n == 42 and pts >= 1:
        return True, n, pts, cx
    return False, n, pts, cx

def leds(ok):
    if ok:
        led_ctrl.set_bottom_led(rm_define.armor_bottom_all, 0, 255, 0, rm_define.effect_always_on)
        led_ctrl.set_top_led(rm_define.armor_top_all, 0, 255, 0, rm_define.effect_always_on)
    else:
        led_ctrl.set_bottom_led(rm_define.armor_bottom_all, 255, 0, 0, rm_define.effect_always_on)
        led_ctrl.set_top_led(rm_define.armor_top_all, 255, 0, 0, rm_define.effect_always_on)

def start():
    print("======== Line Pitch + Full Dump ========")
    print("# LINE_PITCH_TEST_VERSION=1.4.0 stamp=2026-08-04 13:55:00")
    print("[LPT] free mode; absolute pitch sweep; RmList dump")
    print("[LPT] Understanding target:")
    print("[LPT]   no line -> len often ~2")
    print("[LPT]   has line -> len 42, [2]=num_points, [19]=line_x in 0..1")

    robot_ctrl.set_mode(rm_define.robot_mode_free)
    chassis_ctrl.stop()
    try:
        gimbal_ctrl.set_rotate_speed(200)
    except Exception:
        pass
    vision_ctrl.enable_detection(rm_define.vision_detection_line)
    vision_ctrl.line_follow_color_set(rm_define.line_follow_color_blue)
    media_ctrl.exposure_value_update(rm_define.exposure_value_small)
    try:
        gimbal_ctrl.yaw_ctrl(0)
    except Exception:
        pass
    time.sleep(0.4)

    results = []
    pi = 0
    while pi < len(PITCH_LIST):
        p = PITCH_LIST[pi]
        print("[LPT] === pitch_cmd=%d ===" % p)
        try:
            gimbal_ctrl.pitch_ctrl(p)
        except Exception:
            print("[LPT] pitch_ctrl FAIL %d" % p)
            pi = pi + 1
            continue
        time.sleep(SETTLE)
        try:
            act = gimbal_ctrl.get_axis_angle(rm_define.gimbal_axis_pitch)
        except Exception:
            act = p * 1.0

        hits = 0
        dumped = False
        last_n = 0
        last_pts = 0
        last_cx = 0.5
        s = 0
        while s < SAMPLES:
            info, kind = fetch_rmlist()
            if DUMP_FULL_ONCE and dumped == False:
                dump_full(info, kind, "pitch=%d" % p)
                dumped = True
            ok, n, pts, cx = official_see(info)
            last_n = n
            last_pts = pts
            last_cx = cx
            if ok:
                hits = hits + 1
            time.sleep(SAMPLE_DT)
            s = s + 1

        ratio = (1.0 * hits) / SAMPLES
        see = hits > 0
        leds(see)
        print(
            "[LPT] RESULT pitch_cmd=%d actual=%.1f hits=%d/%d ratio=%.2f n=%d pts=%d cx=%.3f %s"
            % (p, act, hits, SAMPLES, ratio, last_n, last_pts, last_cx, "SEE" if see else "NO")
        )
        results.append((p, act, hits, ratio, last_n, last_pts, last_cx, see))
        pi = pi + 1

    print("[LPT] ======== SUMMARY (absolute pitch) ========")
    deepest = None
    j = 0
    while j < len(results):
        p, act, hits, ratio, n, pts, cx, see = results[j]
        tag = "SEE" if see else "NO "
        print(
            "[LPT] sum pitch=%3d actual=%5.1f hits=%2d/%d pts=%d cx=%.3f %s"
            % (p, act, hits, SAMPLES, pts, cx, tag)
        )
        if see:
            if deepest is None or p < deepest:
                deepest = p
        j = j + 1

    print("[LPT] ---- 对 get_line_detection_info 的理解（结合本机日志）----")
    print("[LPT] 1) 须 RmList(vision_ctrl.get_line_detection_info())")
    print("[LPT] 2) 无线：len 常为 2 左右，[2] 无效或 0")
    print("[LPT] 3) 有线：len==42，[2]=线点数(>=1)，[19]=横向位置 0~1（0.5居中）")
    print("[LPT] 4) 其余下标多为沿线采样点坐标，官方循线主要用 [2]+[19]")
    print("[LPT] 5) 硬件 pitch 下限约 -20；更负通常无法 pitch_ctrl")

    if deepest is None:
        print("[LPT] RECOMMEND: 本轮无 SEE — 检查是否 1.4.0、线是否在画面、颜色 blue")
        led_ctrl.set_bottom_led(rm_define.armor_bottom_all, 255, 0, 0, rm_define.effect_flash)
        try:
            led_ctrl.set_flash(rm_define.armor_all, 4)
        except Exception:
            pass
    else:
        print(
            "[LPT] RECOMMEND deepest SEE pitch=%d (越负越低头；可作 PITCH_LINE 候选)"
            % deepest
        )
        try:
            gimbal_ctrl.pitch_ctrl(deepest)
        except Exception:
            pass
        leds(True)
    print("======== Line Pitch Test done ========")
