# PERSON_DETECT_TEST_VERSION=1.0.0 stamp=2026-08-04 14:20:00
# -*- coding: utf-8 -*-
#
# S1 行人识别测试（单文件，整段粘贴进 App 实验室）
#
# 严格按官方文档/实验室 API，不做巡线、不射击、不复杂状态机：
#   robot_mode_free
#   vision_ctrl.enable_detection(rm_define.vision_detection_people)
#   vision_ctrl.get_people_detection_info()   # 或 RmList 包一层
#   vision_ctrl.check_condition(rm_define.cond_recognized_people)  # 对照
#   识别到人 → 绿灯 + 识别成功音；无人 → 红灯
#
# 用法：
#   1) 只跑本文件（不要和 line_guard 混）
#   2) 控制台必须出现 VERSION 1.0.0
#   3) 人走进/走出画面，看灯与日志；可截 FPV + 控制台
#
# 关于多线程：
#   S1 App 实验室 Python 一般不提供可靠多线程；官方也以单循环 + 阻塞/非阻塞 API 为主。
#   本测试无 threading。射击若以后要「不挡主循环」，应优先用非阻塞的
#   gun_ctrl.fire_continuous() + 定时 gun_ctrl.stop()，而不是另开线程。

# =============================================================================
# CONFIG
# =============================================================================
LOOP_DT = 0.1
# 略抬头看人（巡线用低头；本测试不低头）
PITCH_LOOK = 10
# 连续多少次「有人」才亮绿/播音，减轻闪一下
HIT_NEED = 3
# 连续多少次「无人」才灭成红
MISS_NEED = 5
# 有人时多久最多播一次音（秒）
SFX_COOLDOWN = 1.5
# 是否用 RmList 包一层（与线识别官方习惯一致，两种都打日志）
TRY_RMLIST = True
# 每隔多少秒完整 dump 一次返回值
DUMP_PERIOD = 1.0

# =============================================================================
def now_s():
    return tools.run_time_of_program()

def leds(r, g, b):
    led_ctrl.set_bottom_led(rm_define.armor_bottom_all, r, g, b, rm_define.effect_always_on)
    led_ctrl.set_top_led(rm_define.armor_top_all, r, g, b, rm_define.effect_always_on)

def leds_person():
    # 绿 = 有人
    leds(0, 255, 0)

def leds_empty():
    # 红 = 无人
    leds(255, 0, 0)

def sfx_person():
    try:
        media_ctrl.play_sound(rm_define.media_sound_recognize_success)
    except Exception:
        try:
            media_ctrl.play_sound(rm_define.media_sound_solmization_2C)
        except Exception:
            pass

def fetch_people_info():
    """
    官方：vision_ctrl.get_people_detection_info()
    可选 RmList 包装后返回 (info, kind)
    """
    raw = vision_ctrl.get_people_detection_info()
    if TRY_RMLIST:
        try:
            return RmList(raw), "RmList"
        except Exception:
            return raw, "list"
    return raw, "list"

def info_len(info):
    try:
        return len(info)
    except Exception:
        return -1

def info_get(info, i):
    try:
        return info[i]
    except Exception:
        return None

def dump_people(info, kind, tag):
    n = info_len(info)
    print("[PDT] ---- DUMP %s kind=%s len=%d ----" % (tag, kind, n))
    i = 0
    line = ""
    # 最多打前 24 个，避免刷爆
    limit = n
    if limit > 24:
        limit = 24
    while i < limit:
        line = line + "[%d]=%s " % (i, str(info_get(info, i)))
        if (i + 1) % 6 == 0:
            print("[PDT]   " + line)
            line = ""
        i = i + 1
    if line != "":
        print("[PDT]   " + line)
    if n > 24:
        print("[PDT]   ... (%d more)" % (n - 24))
    print("[PDT] ---- END DUMP ----")

def parse_people(info):
    """
    常见返回（实验室/社区，两种排布都试）：
      A) [n, x, y, w, h, ...]
      B) [?, n, x, y, w, h, ...]
    n>=1 且框在画面内视为有人。
    返回 (ok, n_person, x, y, w, h, note)
    """
    nlen = info_len(info)
    if nlen < 1:
        return False, 0, 0.5, 0.5, 0.0, 0.0, "empty"

    # 排布 A
    try:
        n0 = int(info_get(info, 0))
        if n0 >= 1 and nlen >= 5:
            x = float(info_get(info, 1))
            y = float(info_get(info, 2))
            w = float(info_get(info, 3))
            h = float(info_get(info, 4))
            if x >= 0.0 and x <= 1.0 and y >= 0.0 and y <= 1.0:
                return True, n0, x, y, w, h, "layout_A"
    except Exception:
        pass

    # 排布 B
    try:
        if nlen >= 6:
            n1 = int(info_get(info, 1))
            if n1 >= 1:
                x = float(info_get(info, 2))
                y = float(info_get(info, 3))
                w = float(info_get(info, 4))
                h = float(info_get(info, 5))
                if x >= 0.0 and x <= 1.0 and y >= 0.0 and y <= 1.0:
                    return True, n1, x, y, w, h, "layout_B"
    except Exception:
        pass

    # 官方条件块对照（不解析坐标）
    try:
        if vision_ctrl.check_condition(rm_define.cond_recognized_people):
            return True, 1, 0.5, 0.5, 0.0, 0.0, "cond_only"
    except Exception:
        pass

    return False, 0, 0.5, 0.5, 0.0, 0.0, "no_person"

def start():
    print("======== Person Detect Test ========")
    print("# PERSON_DETECT_TEST_VERSION=1.0.0 stamp=2026-08-04 14:20:00")
    print("[PDT] official APIs only: enable people + get_people_detection_info")
    print("[PDT] NO line / NO gun / NO state machine")

    robot_ctrl.set_mode(rm_define.robot_mode_free)
    chassis_ctrl.stop()
    try:
        gimbal_ctrl.set_rotate_speed(120)
    except Exception:
        pass
    try:
        gimbal_ctrl.yaw_ctrl(0)
        gimbal_ctrl.pitch_ctrl(PITCH_LOOK)
    except Exception:
        pass
    time.sleep(0.4)

    # 只开行人，关掉线（避免干扰）
    vision_ctrl.enable_detection(rm_define.vision_detection_people)
    try:
        vision_ctrl.disable_detection(rm_define.vision_detection_line)
    except Exception:
        pass

    leds_empty()
    hit = 0
    miss = 0
    person_on = False
    last_sfx = 0.0
    last_dump = 0.0
    frame = 0

    print("[PDT] loop start pitch=%d HIT_NEED=%d MISS_NEED=%d" % (PITCH_LOOK, HIT_NEED, MISS_NEED))

    while True:
        info, kind = fetch_people_info()
        ok, np, x, y, w, h, note = parse_people(info)

        # 条件块再读一次对照
        cond = False
        try:
            cond = vision_ctrl.check_condition(rm_define.cond_recognized_people)
        except Exception:
            cond = False

        t = now_s()
        if t - last_dump >= DUMP_PERIOD:
            dump_people(info, kind, "t=%.1f" % t)
            print(
                "[PDT] parse ok=%s note=%s n=%s xywh=(%.2f,%.2f,%.2f,%.2f) cond=%s hit=%d miss=%d"
                % (str(ok), note, str(np), x, y, w, h, str(cond), hit, miss)
            )
            last_dump = t

        if ok or cond:
            hit = hit + 1
            miss = 0
        else:
            miss = miss + 1
            hit = 0

        if person_on == False and hit >= HIT_NEED:
            person_on = True
            leds_person()
            if t - last_sfx >= SFX_COOLDOWN:
                sfx_person()
                last_sfx = t
            print("[PDT] >>> PERSON ON note=%s cond=%s xy=(%.2f,%.2f) wh=(%.2f,%.2f)" % (
                note, str(cond), x, y, w, h
            ))
        elif person_on and miss >= MISS_NEED:
            person_on = False
            leds_empty()
            print("[PDT] <<< PERSON OFF")

        frame = frame + 1
        time.sleep(LOOP_DT)
