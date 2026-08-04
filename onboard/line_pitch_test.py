# LINE_PITCH_TEST_VERSION=1.3.0 stamp=2026-08-04 13:44:15
# -*- coding: utf-8 -*-
# 线识别冒烟测试：几乎照抄你能跑通的官方程序，只加日志与灯。
# 控制台必须出现 VERSION 1.3.0，否则是旧粘贴。

def start():
    print("======== Official-style Line Smoke ========")
    print("# LINE_PITCH_TEST_VERSION=1.3.0 stamp=2026-08-04 13:44:15")
    print("[LPT] same path as DJI sample: RmList + len==42 + [2] + [19]")

    pid_line = None
    try:
        pid_line = PIDCtrl()
        pid_line.set_ctrl_params(330, 0, 28)
        print("[LPT] PIDCtrl ok")
    except Exception:
        try:
            pid_line = rm_ctrl.PIDCtrl()
            pid_line.set_ctrl_params(330, 0, 28)
            print("[LPT] rm_ctrl.PIDCtrl ok")
        except Exception:
            print("[LPT] PIDCtrl missing, use P=330")

    robot_ctrl.set_mode(rm_define.robot_mode_chassis_follow)
    try:
        gimbal_ctrl.yaw_ctrl(0)
        gimbal_ctrl.pitch_ctrl(0)
    except Exception:
        pass
    time.sleep(0.3)
    gimbal_ctrl.rotate_with_degree(rm_define.gimbal_down, 20)
    time.sleep(0.4)
    vision_ctrl.enable_detection(rm_define.vision_detection_line)
    vision_ctrl.line_follow_color_set(rm_define.line_follow_color_blue)

    hits = 0
    i = 0
    while i < 80:
        list_LineList = RmList(vision_ctrl.get_line_detection_info())
        n = len(list_LineList)
        pts = -1
        cx = -1.0
        try:
            pts = int(list_LineList[2])
        except Exception:
            pts = -1
        try:
            cx = float(list_LineList[19])
        except Exception:
            cx = -1.0

        if n == 42 and pts >= 1:
            hits = hits + 1
            variable_x = list_LineList[19]
            err = variable_x - 0.5
            if pid_line is not None:
                pid_line.set_error(err)
                out = pid_line.get_output()
            else:
                out = err * 330.0
            gimbal_ctrl.rotate_with_speed(out, 0)
            chassis_ctrl.set_trans_speed(0.2)
            chassis_ctrl.move(0)
            led_ctrl.set_bottom_led(rm_define.armor_bottom_all, 0, 255, 0, rm_define.effect_always_on)
            led_ctrl.set_top_led(rm_define.armor_top_all, 0, 255, 0, rm_define.effect_always_on)
            if i % 10 == 0:
                print("[LPT] SEE n=%d pts=%d cx=%.3f out=%.1f hits=%d" % (n, pts, float(variable_x), out, hits))
        else:
            gimbal_ctrl.rotate_with_speed(0, 0)
            chassis_ctrl.stop()
            led_ctrl.set_bottom_led(rm_define.armor_bottom_all, 255, 0, 0, rm_define.effect_always_on)
            led_ctrl.set_top_led(rm_define.armor_top_all, 255, 0, 0, rm_define.effect_always_on)
            if i % 10 == 0:
                print("[LPT] NO  n=%d pts=%d cx=%.3f hits=%d" % (n, pts, cx, hits))
        time.sleep(0.05)
        i = i + 1

    chassis_ctrl.stop()
    gimbal_ctrl.rotate_with_speed(0, 0)
    print("[LPT] DONE hits=%d/80 (official path)" % hits)
    if hits > 10:
        print("[LPT] OK — RmList path works; use line_guard 1.14")
        led_ctrl.set_bottom_led(rm_define.armor_bottom_all, 0, 255, 0, rm_define.effect_always_on)
    else:
        print("[LPT] FAIL — even official path no line; check enable/firmware")
        led_ctrl.set_bottom_led(rm_define.armor_bottom_all, 255, 0, 0, rm_define.effect_flash)
    print("======== done ========")
