# LINE_PITCH_TEST_VERSION=1.2.0 stamp=2026-08-04 13:45:00
# -*- coding: utf-8 -*-
# S1 线识别诊断（单文件）。若控制台不是 1.2.0，说明粘贴的是旧代码。
#
# 几乎照抄 DJI 实验室 Line Follower：
#   enable_detection(line) + line_follow_color_set + get_line_detection_info
#   判定：len==42 且 [2]>=1，x=[19]
# 额外：打印 RAW；试 blue/red/green；试曝光 small/medium；俯仰 0~-20

PITCH_LIST = [0, -10, -15, -20]
COLORS = [
    ("blue", "line_follow_color_blue"),
    ("red", "line_follow_color_red"),
    ("green", "line_follow_color_green"),
]
EXPS = ["small", "medium"]
SAMPLES = 15
SAMPLE_DT = 0.05
SETTLE = 0.5

def set_exp(name):
    if name == "small":
        media_ctrl.exposure_value_update(rm_define.exposure_value_small)
    elif name == "large":
        media_ctrl.exposure_value_update(rm_define.exposure_value_large)
    else:
        media_ctrl.exposure_value_update(rm_define.exposure_value_medium)

def set_color(name):
    if name == "red":
        vision_ctrl.line_follow_color_set(rm_define.line_follow_color_red)
    elif name == "green":
        vision_ctrl.line_follow_color_set(rm_define.line_follow_color_green)
    else:
        vision_ctrl.line_follow_color_set(rm_define.line_follow_color_blue)

def fetch_info():
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

def dump_raw(info, kind, tag):
    n = ilen(info)
    print(
        "[LPT] RAW %s kind=%s n=%s v0=%s v1=%s v2=%s v3=%s v18=%s v19=%s v20=%s"
        % (
            tag,
            kind,
            str(n),
            str(iget(info, 0)),
            str(iget(info, 1)),
            str(iget(info, 2)),
            str(iget(info, 3)),
            str(iget(info, 18)),
            str(iget(info, 19)),
            str(iget(info, 20)),
        )
    )

def official_ok(info):
    """DJI 示例：len==42 and [2]>=1 → 有线。"""
    n = ilen(info)
    pts = 0
    try:
        if iget(info, 2) is not None:
            pts = int(iget(info, 2))
    except Exception:
        pts = 0
    cx = 0.5
    try:
        if n > 19 and iget(info, 19) is not None:
            cx = float(iget(info, 19))
    except Exception:
        cx = 0.5
    # 官方严格
    if n == 42 and pts >= 1:
        return True, n, pts, cx, "strict42"
    # 放宽 n
    if n >= 42 and pts >= 1:
        return True, n, pts, cx, "ge42"
    if pts >= 1 and n > 19:
        return True, n, pts, cx, "pts_only"
    return False, n, pts, cx, "fail"

def leds(ok):
    if ok:
        led_ctrl.set_bottom_led(rm_define.armor_bottom_all, 0, 255, 0, rm_define.effect_always_on)
        led_ctrl.set_top_led(rm_define.armor_top_all, 0, 255, 0, rm_define.effect_always_on)
    else:
        led_ctrl.set_bottom_led(rm_define.armor_bottom_all, 255, 0, 0, rm_define.effect_always_on)
        led_ctrl.set_top_led(rm_define.armor_top_all, 255, 0, 0, rm_define.effect_always_on)

def start():
    print("======== Line Pitch Test ========")
    print("# LINE_PITCH_TEST_VERSION=1.2.0 stamp=2026-08-04 13:45:00")
    print("[LPT] MUST see VERSION 1.2.0 above — else old paste")
    print("[LPT] tape on white paper in front of camera, free mode")

    robot_ctrl.set_mode(rm_define.robot_mode_free)
    chassis_ctrl.stop()
    try:
        gimbal_ctrl.set_rotate_speed(200)
    except Exception:
        pass
    # 只开线识别（不跟人）
    vision_ctrl.enable_detection(rm_define.vision_detection_line)
    try:
        vision_ctrl.disable_detection(rm_define.vision_detection_people)
    except Exception:
        pass
    try:
        gimbal_ctrl.yaw_ctrl(0)
    except Exception:
        pass
    time.sleep(0.5)

    any_hit = False
    best = None

    pi = 0
    while pi < len(PITCH_LIST):
        p = PITCH_LIST[pi]
        try:
            gimbal_ctrl.pitch_ctrl(p)
        except Exception:
            print("[LPT] pitch FAIL %d" % p)
            pi = pi + 1
            continue
        time.sleep(SETTLE)
        try:
            act = gimbal_ctrl.get_axis_angle(rm_define.gimbal_axis_pitch)
        except Exception:
            act = p * 1.0

        ci = 0
        while ci < len(COLORS):
            cname = COLORS[ci][0]
            set_color(cname)
            time.sleep(0.15)
            ei = 0
            while ei < len(EXPS):
                en = EXPS[ei]
                set_exp(en)
                time.sleep(0.2)

                hits = 0
                dumped = False
                last_n = 0
                last_pts = 0
                last_cx = 0.5
                last_note = ""
                kind = ""
                s = 0
                while s < SAMPLES:
                    info, kind = fetch_info()
                    if dumped == False:
                        dump_raw(info, kind, "p=%d c=%s e=%s" % (p, cname, en))
                        dumped = True
                    ok, n, pts, cx, note = official_ok(info)
                    last_n = n
                    last_pts = pts
                    last_cx = cx
                    last_note = note
                    if ok:
                        hits = hits + 1
                    time.sleep(SAMPLE_DT)
                    s = s + 1

                ratio = (1.0 * hits) / SAMPLES
                see = hits > 0
                if see:
                    any_hit = True
                    if best is None or p < best[0]:
                        best = (p, cname, en, ratio)
                leds(see)
                print(
                    "[LPT] pitch=%d act=%.1f color=%s exp=%s hits=%d/%d n=%d pts=%d cx=%.2f note=%s kind=%s %s"
                    % (
                        p,
                        act,
                        cname,
                        en,
                        hits,
                        SAMPLES,
                        last_n,
                        last_pts,
                        last_cx,
                        last_note,
                        kind,
                        "SEE" if see else "NO",
                    )
                )
                ei = ei + 1
            ci = ci + 1
        pi = pi + 1

    print("[LPT] ======== SUMMARY ========")
    if any_hit == False:
        print("[LPT] RECOMMEND: NONE — 所有组合 pts 仍像 0 / 无 hit")
        print("[LPT] 若 RAW 里 v2 一直是 0 或 None：S1 视觉未检出线（非解析下标问题）")
        print("[LPT] 若从未出现 VERSION 1.2.0：App 里仍是旧测试代码，请整文件重贴")
        print("[LPT] 建议 App 图形编程：使能线识别+颜色蓝+识别到线则亮灯，交叉验证硬件/固件")
        led_ctrl.set_bottom_led(rm_define.armor_bottom_all, 255, 0, 0, rm_define.effect_flash)
        led_ctrl.set_top_led(rm_define.armor_top_all, 255, 0, 0, rm_define.effect_flash)
        try:
            led_ctrl.set_flash(rm_define.armor_all, 5)
        except Exception:
            pass
    else:
        print(
            "[LPT] RECOMMEND pitch=%d color=%s exp=%s ratio=%.2f"
            % (best[0], best[1], best[2], best[3])
        )
        try:
            gimbal_ctrl.pitch_ctrl(best[0])
        except Exception:
            pass
        set_color(best[1])
        leds(True)
    print("======== Line Pitch Test done ========")
