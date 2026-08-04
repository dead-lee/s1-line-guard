# LINE_PITCH_TEST_VERSION=1.0.0 stamp=2026-08-04 13:25:00
# -*- coding: utf-8 -*-
# S1 蓝线识别 × 云台俯仰 测试（单文件粘贴进 App 实验室）
#
# 用途：
#   车身压在蓝线上，测试不同「下压」俯仰角能否稳定看到线。
#   在控制台打印每个角度的识别结果，并推荐「下压尽量多、且眼前能看到线」的角度。
#
# 使用：
#   1) 车压蓝线 / 线在车头正前方地面
#   2) 全选本文件 → App 实验室 Python → 运行
#   3) 看控制台每档 pitch 的 hit/pts/cx；结束有推荐角度
#
# 俯仰范围（S1 文档约 -20~+35）：数值越小越低头看地。

# =============================================================================
# CONFIG
# =============================================================================
# 待测俯仰（度）：从略平视 → 尽量低头（-20 为硬件下限附近）
# 顺序：先浅后深，最后再汇总「越负越好」的推荐
PITCH_LIST = [10, 5, 0, -5, -10, -12, -15, -17, -18, -19, -20]

# 每个角度：到位等待 + 采样帧数
SETTLE_S = 0.45
SAMPLES = 20
SAMPLE_DT = 0.05

# 判定「能看见线」：采样中成功帧比例
HIT_OK_RATIO = 0.40
# 推荐：在 hit 达标中选 pitch 最小（下压最多）的一档
# 颜色
LINE_COLOR_BLUE = True

# =============================================================================
# LINE PARSE（与 line_guard 一致：官方 len/点数/[19]）
# =============================================================================
def line_parse(info):
    """返回 (ok, cx, n, pts)。"""
    try:
        n = len(info)
    except Exception:
        return False, 0.5, 0, 0
    pts = 0
    try:
        if n > 2 and info[2] is not None:
            pts = int(info[2])
    except Exception:
        pts = 0
    if n >= 42 and pts >= 1:
        try:
            cx = float(info[19])
            if cx >= 0.0 and cx <= 1.0:
                return True, cx, n, pts
        except Exception:
            pass
        for idx in (18, 20, 17, 21, 3, 5):
            try:
                if n > idx:
                    cx = float(info[idx])
                    if cx >= 0.0 and cx <= 1.0:
                        return True, cx, n, pts
            except Exception:
                pass
    if n > 19 and pts >= 1:
        try:
            cx = float(info[19])
            if cx >= 0.0 and cx <= 1.0:
                return True, cx, n, pts
        except Exception:
            pass
    return False, 0.5, n, pts

def sample_line_at_pitch(pitch_deg):
    """
    固定俯仰采样 SAMPLES 帧。
    返回 dict: hits, n_avg, pts_avg, cx_avg, ok_ratio
    """
    hits = 0
    n_sum = 0
    pts_sum = 0
    cx_sum = 0.0
    cx_n = 0
    i = 0
    while i < SAMPLES:
        info = vision_ctrl.get_line_detection_info()
        ok, cx, n, pts = line_parse(info)
        n_sum = n_sum + n
        pts_sum = pts_sum + pts
        if ok:
            hits = hits + 1
            cx_sum = cx_sum + cx
            cx_n = cx_n + 1
        time.sleep(SAMPLE_DT)
        i = i + 1
    ratio = 0.0
    if SAMPLES > 0:
        ratio = (1.0 * hits) / SAMPLES
    n_avg = 0.0
    pts_avg = 0.0
    cx_avg = 0.5
    if SAMPLES > 0:
        n_avg = (1.0 * n_sum) / SAMPLES
        pts_avg = (1.0 * pts_sum) / SAMPLES
    if cx_n > 0:
        cx_avg = cx_sum / cx_n
    return {
        "hits": hits,
        "ratio": ratio,
        "n_avg": n_avg,
        "pts_avg": pts_avg,
        "cx_avg": cx_avg,
    }

def leds_show(ok):
    """绿=本档看见线，红=看不见。"""
    if ok:
        led_ctrl.set_bottom_led(rm_define.armor_bottom_all, 0, 255, 0, rm_define.effect_always_on)
        led_ctrl.set_top_led(rm_define.armor_top_all, 0, 255, 0, rm_define.effect_always_on)
    else:
        led_ctrl.set_bottom_led(rm_define.armor_bottom_all, 255, 0, 0, rm_define.effect_always_on)
        led_ctrl.set_top_led(rm_define.armor_top_all, 255, 0, 0, rm_define.effect_always_on)

def start():
    print("======== Line Pitch Test start ========")
    print("# LINE_PITCH_TEST_VERSION=1.0.0 stamp=2026-08-04 13:25:00")
    print("[LPT] free mode, blue line, yaw=0, sweep pitch down")
    robot_ctrl.set_mode(rm_define.robot_mode_free)
    chassis_ctrl.stop()
    try:
        gimbal_ctrl.set_rotate_speed(200)
    except Exception:
        pass
    vision_ctrl.enable_detection(rm_define.vision_detection_line)
    if LINE_COLOR_BLUE:
        vision_ctrl.line_follow_color_set(rm_define.line_follow_color_blue)
    media_ctrl.exposure_value_update(rm_define.exposure_value_medium)

    # 航向归中，只改俯仰
    try:
        gimbal_ctrl.yaw_ctrl(0)
    except Exception:
        pass
    time.sleep(0.3)

    results = []
    i = 0
    while i < len(PITCH_LIST):
        p = PITCH_LIST[i]
        print("[LPT] --- pitch=%d deg ---" % p)
        try:
            gimbal_ctrl.pitch_ctrl(p)
        except Exception:
            print("[LPT] pitch_ctrl(%d) FAIL" % p)
            i = i + 1
            continue
        time.sleep(SETTLE_S)
        try:
            actual = gimbal_ctrl.get_axis_angle(rm_define.gimbal_axis_pitch)
        except Exception:
            actual = p
        r = sample_line_at_pitch(p)
        ok = r["ratio"] >= HIT_OK_RATIO
        leds_show(ok)
        print(
            "[LPT] pitch_cmd=%d actual=%.1f hits=%d/%d ratio=%.2f pts=%.1f n=%.1f cx=%.2f %s"
            % (
                p,
                actual,
                r["hits"],
                SAMPLES,
                r["ratio"],
                r["pts_avg"],
                r["n_avg"],
                r["cx_avg"],
                "SEE_LINE" if ok else "NO_LINE",
            )
        )
        results.append((p, r, ok, actual))
        time.sleep(0.2)
        i = i + 1

    # 汇总：能看见线的里面，选下压最多（pitch 最小）
    print("[LPT] ======== SUMMARY ========")
    best_p = None
    best_ratio = 0.0
    j = 0
    while j < len(results):
        p, r, ok, actual = results[j]
        tag = "OK" if ok else "--"
        print(
            "[LPT] sum pitch=%d actual=%.1f ratio=%.2f pts=%.1f %s"
            % (p, actual, r["ratio"], r["pts_avg"], tag)
        )
        if ok:
            if best_p is None or p < best_p:
                best_p = p
                best_ratio = r["ratio"]
        j = j + 1

    if best_p is None:
        print("[LPT] RECOMMEND: NONE — 所有角度都看不到线")
        print("[LPT] 检查：蓝胶带/颜色/曝光/是否在画面中/是否 enable line")
        # 红闪提示失败
        led_ctrl.set_bottom_led(rm_define.armor_bottom_all, 255, 0, 0, rm_define.effect_flash)
        led_ctrl.set_top_led(rm_define.armor_top_all, 255, 0, 0, rm_define.effect_flash)
        try:
            led_ctrl.set_flash(rm_define.armor_all, 4)
        except Exception:
            pass
    else:
        print(
            "[LPT] RECOMMEND pitch=%d (most downward among SEE_LINE, ratio=%.2f)"
            % (best_p, best_ratio)
        )
        print("[LPT] 可把 line_guard 的 PITCH_LINE 改成 %d 再测巡线" % best_p)
        # 停在推荐角度，绿灯
        try:
            gimbal_ctrl.pitch_ctrl(best_p)
        except Exception:
            pass
        leds_show(True)

    print("======== Line Pitch Test done ========")
