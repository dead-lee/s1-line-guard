# LINE_PITCH_TEST_VERSION=1.1.0 stamp=2026-08-04 13:35:00
# -*- coding: utf-8 -*-
# S1 蓝线识别 × 云台俯仰 诊断测试（单文件粘贴进 App 实验室）
#
# 用途：车压蓝线，扫不同俯仰；打印「原始 API 数据 + 解析结果」
# 判断是「算法真没看见线」还是「我们解析下标错了」。
#
# 官方实验室示例（DJI）：
#   vision.enable_detection(line)
#   vision.line_follow_color_set(blue)
#   LineList = RmList(get_line_detection_info())
#   len==42 且 LineList[2]>=1 → 有线，x=LineList[19]
#
# 说明：电工胶带蓝偏深，白瓷砖反光强时，S1 常检不出（pts=0）。
# 18mm 宽度在推荐 15~25mm 内，宽度一般不是主因。

# =============================================================================
# CONFIG
# =============================================================================
PITCH_LIST = [5, 0, -5, -10, -15, -18, -20]
SETTLE_S = 0.5
SAMPLES = 12
SAMPLE_DT = 0.05
HIT_OK_RATIO = 0.35
# 每个俯仰再试三种曝光（文档：large/medium/small）
EXPOSURE_NAMES = ["medium", "small", "large"]

# =============================================================================
# 原始数据 / 解析
# =============================================================================
def exp_set(name):
    if name == "large":
        media_ctrl.exposure_value_update(rm_define.exposure_value_large)
    elif name == "small":
        media_ctrl.exposure_value_update(rm_define.exposure_value_small)
    else:
        media_ctrl.exposure_value_update(rm_define.exposure_value_medium)

def get_info_raw_list():
    """取线识别原始列表；优先包成 RmList（与官方示例一致）。"""
    raw = vision_ctrl.get_line_detection_info()
    try:
        return RmList(raw)
    except Exception:
        return raw

def list_len(info):
    try:
        return len(info)
    except Exception:
        return -1

def list_get(info, idx):
    try:
        return info[idx]
    except Exception:
        return None

def dump_raw(info, tag):
    """打印关键下标，便于对照官方 [2]=点数 [19]=x。"""
    n = list_len(info)
    a0 = list_get(info, 0)
    a1 = list_get(info, 1)
    a2 = list_get(info, 2)
    a3 = list_get(info, 3)
    a18 = list_get(info, 18)
    a19 = list_get(info, 19)
    a20 = list_get(info, 20)
    print(
        "[LPT] %s RAW n=%s [0]=%s [1]=%s [2]=%s [3]=%s [18]=%s [19]=%s [20]=%s"
        % (tag, str(n), str(a0), str(a1), str(a2), str(a3), str(a18), str(a19), str(a20))
    )

def parse_official(info):
    """
    严格按官方：len==42 且 [2]>=1，cx=[19]。
    返回 (ok, cx, n, pts, note)
    """
    n = list_len(info)
    pts = 0
    try:
        v2 = list_get(info, 2)
        if v2 is not None:
            pts = int(v2)
    except Exception:
        pts = 0
    # 官方判断
    if n == 42 and pts >= 1:
        try:
            cx = float(list_get(info, 19))
            return True, cx, n, pts, "official_42"
        except Exception:
            return False, 0.5, n, pts, "official_42_no_cx"
    # 放宽：n>=42
    if n >= 42 and pts >= 1:
        try:
            cx = float(list_get(info, 19))
            return True, cx, n, pts, "n>=42"
        except Exception:
            return False, 0.5, n, pts, "n>=42_no_cx"
    # 兼容：有的固件 n 不是 42 但 [2]>=1
    if pts >= 1 and n > 19:
        try:
            cx = float(list_get(info, 19))
            return True, cx, n, pts, "pts_and_19"
        except Exception:
            pass
    return False, 0.5, n, pts, "no_line"

def sample_once(pitch_cmd, exp_name):
    hits = 0
    last_note = ""
    last_n = 0
    last_pts = 0
    last_cx = 0.5
    dumped = False
    i = 0
    while i < SAMPLES:
        info = get_info_raw_list()
        if dumped == False:
            dump_raw(info, "pitch=%d exp=%s" % (pitch_cmd, exp_name))
            dumped = True
        ok, cx, n, pts, note = parse_official(info)
        last_note = note
        last_n = n
        last_pts = pts
        last_cx = cx
        if ok:
            hits = hits + 1
        time.sleep(SAMPLE_DT)
        i = i + 1
    ratio = 0.0
    if SAMPLES > 0:
        ratio = (1.0 * hits) / SAMPLES
    return hits, ratio, last_n, last_pts, last_cx, last_note

def leds_show(ok):
    if ok:
        led_ctrl.set_bottom_led(rm_define.armor_bottom_all, 0, 255, 0, rm_define.effect_always_on)
        led_ctrl.set_top_led(rm_define.armor_top_all, 0, 255, 0, rm_define.effect_always_on)
    else:
        led_ctrl.set_bottom_led(rm_define.armor_bottom_all, 255, 0, 0, rm_define.effect_always_on)
        led_ctrl.set_top_led(rm_define.armor_top_all, 255, 0, 0, rm_define.effect_always_on)

def start():
    print("======== Line Pitch Test v1.1 ========")
    print("# LINE_PITCH_TEST_VERSION=1.1.0 stamp=2026-08-04 13:35:00")
    print("[LPT] blue electrical tape? white tile? width~18mm")
    print("[LPT] API: enable_detection(line)+line_follow_color_blue + get_line_detection_info")
    robot_ctrl.set_mode(rm_define.robot_mode_free)
    chassis_ctrl.stop()
    try:
        gimbal_ctrl.set_rotate_speed(200)
    except Exception:
        pass
    vision_ctrl.enable_detection(rm_define.vision_detection_line)
    vision_ctrl.line_follow_color_set(rm_define.line_follow_color_blue)
    try:
        gimbal_ctrl.yaw_ctrl(0)
    except Exception:
        pass
    time.sleep(0.4)

    any_ok = False
    best_p = None
    best_ratio = 0.0
    pi = 0
    while pi < len(PITCH_LIST):
        p = PITCH_LIST[pi]
        try:
            gimbal_ctrl.pitch_ctrl(p)
        except Exception:
            print("[LPT] pitch_ctrl(%d) FAIL" % p)
            pi = pi + 1
            continue
        time.sleep(SETTLE_S)
        try:
            actual = gimbal_ctrl.get_axis_angle(rm_define.gimbal_axis_pitch)
        except Exception:
            actual = p * 1.0

        ei = 0
        pitch_best_ratio = 0.0
        pitch_ok = False
        while ei < len(EXPOSURE_NAMES):
            en = EXPOSURE_NAMES[ei]
            exp_set(en)
            time.sleep(0.2)
            hits, ratio, n, pts, cx, note = sample_once(p, en)
            see = ratio >= HIT_OK_RATIO
            if see:
                pitch_ok = True
                any_ok = True
            if ratio > pitch_best_ratio:
                pitch_best_ratio = ratio
            if see and (best_p is None or p < best_p):
                best_p = p
                best_ratio = ratio
            print(
                "[LPT] pitch_cmd=%d actual=%.1f exp=%s hits=%d/%d ratio=%.2f n=%d pts=%d cx=%.2f note=%s %s"
                % (
                    p,
                    actual,
                    en,
                    hits,
                    SAMPLES,
                    ratio,
                    n,
                    pts,
                    cx,
                    note,
                    "SEE_LINE" if see else "NO_LINE",
                )
            )
            ei = ei + 1

        leds_show(pitch_ok)
        print(
            "[LPT] pitch=%d best_ratio=%.2f %s"
            % (p, pitch_best_ratio, "SEE" if pitch_ok else "NO")
        )
        pi = pi + 1

    print("[LPT] ======== SUMMARY ========")
    if any_ok == False:
        print("[LPT] RECOMMEND: NONE")
        print("[LPT] 结论：API 在返回列表(常见 n=42)，但 pts=0 → 视觉算法未检出蓝线")
        print("[LPT] 函数用法与官方一致；更可能是胶带颜色/反光/画面里没有「S1 认定的蓝」")
        print("[LPT] 建议：换更鲜艳的蓝标识胶带；减反光；App 里用图形块「识别到线」交叉验证")
        led_ctrl.set_bottom_led(rm_define.armor_bottom_all, 255, 0, 0, rm_define.effect_flash)
        led_ctrl.set_top_led(rm_define.armor_top_all, 255, 0, 0, rm_define.effect_flash)
        try:
            led_ctrl.set_flash(rm_define.armor_all, 4)
        except Exception:
            pass
    else:
        print(
            "[LPT] RECOMMEND pitch=%d ratio=%.2f (most downward among SEE)"
            % (best_p, best_ratio)
        )
        try:
            gimbal_ctrl.pitch_ctrl(best_p)
        except Exception:
            pass
        leds_show(True)
    print("======== Line Pitch Test done ========")
