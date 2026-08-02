# LINE_GUARD_VERSION=1.5.0 stamp=2026-08-02 11:45:06  (paste this whole file; check stamp matches latest)
# -*- coding: utf-8 -*-
# S1 Line Guard v1.5 — 单文件，整段粘贴进 App 实验室 Python
#
# 重要：S1 云台 yaw 约 ±250°，不能连续转满物理 360°。
# 日志卡死：target=195 时 yaw 停在 255，acc 不再增加。
#
# SCAN「一正一反」= 扫到右软限位 + 扫到左软限位（用实际 yaw 判断到位）：
#   正转: rotate 直到 yaw >= +YAW_LIM 或卡住
#   反转: rotate 直到 yaw <= -YAW_LIM 或卡住
# 队列结束后 recenter，再判线。
#
# 打断续扫（人离开后 LOST_SCAN，仍允许中途再 LOCK）：
#   - 正转段被打断: 继续扫到右限 + 再反转到左限
#   - 反转段被打断: 继续扫到左限 + 再完整一正一反
#   - 无断点: 完整一正一反
# 线: 多帧确认；射击: 示警一次 + 连发1s/停1s

# =============================================================================
# CONFIG
# =============================================================================
T_MOVE = 3.0
T_CLEAR = 1.0
PERSON_MISS_NEED = 10
LOOP_DT = 0.05
LOG_HEARTBEAT_S = 1.0

PITCH_LINE = -20
PITCH_SCAN = 5
# 扫速偏快；到位看实际 yaw，不靠 360 累计
SCAN_YAW_SPEED = 240.0
# 软限位（略小于硬件约 ±250，避免顶死）
YAW_LIM = 230.0
YAW_ARRIVE = 8.0
# 连续多少帧 yaw 几乎不动则视为到限位/卡住，结束本段
SCAN_STUCK_FRAMES = 12

LINE_SPEED = 0.35
LINE_PID_KP = 80.0
LINE_PID_OUT_MAX = 80.0
LINE_CONFIRM_FRAMES = 8

AIM_YAW_KP = 90.0
AIM_YAW_KI = 0.0
AIM_YAW_KD = 25.0
AIM_YAW_OUT_MAX = 55.0
AIM_PITCH_KP = 55.0
AIM_PITCH_KI = 0.0
AIM_PITCH_KD = 18.0
AIM_PITCH_OUT_MAX = 35.0
AIM_DEADZONE = 0.08
AIM_OK_ERR = 0.10
PERSON_MIN_W = 0.06
PERSON_MIN_H = 0.08

T_AIM_BEFORE_IR = 1.2
T_BURST_ON = 1.0
T_BURST_OFF = 1.0

ENABLE_FIRE = True
FORCE_NO_LINE = False
FLASH_HZ = 4

# =============================================================================
# STATE
# =============================================================================
STATE_INIT = 0
STATE_PATROL = 1
STATE_SCAN = 2
STATE_LOCK = 3
STATE_FIRE = 4
STATE_LOST_SCAN = 5
STATE_RECOVER = 6

FIRE_PHASE_AIM = 0
FIRE_PHASE_IR_DONE = 1
FIRE_PHASE_BURST_ON = 2
FIRE_PHASE_BURST_OFF = 3

# scan 队列项: (dir, deg) dir=+1 正转(右), -1 反转(左)
g_state = STATE_INIT
g_state_t0 = 0.0
g_no_person_t0 = 0.0
g_person_miss = 0
g_patrol_line_t0 = 0.0
g_last_hb_t = 0.0
g_fire_count = 0
g_fire_phase = FIRE_PHASE_AIM
g_phase_t0 = 0.0
g_ir_done = False
g_iy = 0.0
g_ey_prev = 0.0
g_ip = 0.0
g_ep_prev = 0.0

g_line_hit = 0
g_line_miss = 0

# SCAN 运行时：队列元素为转向 dir (+1 右/-1 左)，扫到对应软限位
g_scan_queue = []
g_scan_qi = 0
g_scan_dir = 1
g_scan_last_yaw = 0.0
g_scan_stuck = 0
# 打断保存：只记转向
g_brk_valid = False
g_brk_dir = 1

# =============================================================================
# LOG / MATH
# =============================================================================
def now_s():
    return tools.run_time_of_program()

def state_name(s):
    if s == STATE_INIT:
        return "INIT"
    if s == STATE_PATROL:
        return "PATROL"
    if s == STATE_SCAN:
        return "SCAN"
    if s == STATE_LOCK:
        return "LOCK"
    if s == STATE_FIRE:
        return "FIRE"
    if s == STATE_LOST_SCAN:
        return "LOST_SCAN"
    if s == STATE_RECOVER:
        return "RECOVER"
    return "S?" + str(s)

def log(msg):
    t = 0.0
    try:
        t = now_s()
    except Exception:
        t = 0.0
    print("[LG t=%.1f %s] %s" % (t, state_name(g_state), msg))

def clamp(v, lo, hi):
    if v < lo:
        return lo
    if v > hi:
        return hi
    return v

def state_age():
    return now_s() - g_state_t0

def phase_age():
    return now_s() - g_phase_t0

def pid_reset_aim():
    global g_iy, g_ey_prev, g_ip, g_ep_prev
    g_iy = 0.0
    g_ey_prev = 0.0
    g_ip = 0.0
    g_ep_prev = 0.0

def pid_step(err, i_acc, e_prev, kp, ki, kd, out_max, dt):
    if abs(err) < AIM_DEADZONE:
        return 0.0, 0.0, err
    i_new = i_acc + err * dt
    i_new = clamp(i_new, -1.0, 1.0)
    if dt > 0.0001:
        d = (err - e_prev) / dt
    else:
        d = 0.0
    out = kp * err + ki * i_new + kd * d
    out = clamp(out, -out_max, out_max)
    return out, i_new, err

def get_yaw():
    return gimbal_ctrl.get_axis_angle(rm_define.gimbal_axis_yaw)

# =============================================================================
# LED
# =============================================================================
def leds_normal():
    led_ctrl.set_bottom_led(rm_define.armor_bottom_all, 0, 80, 255, rm_define.effect_always_on)
    led_ctrl.set_top_led(rm_define.armor_top_all, 0, 80, 255, rm_define.effect_always_on)

def leds_alert_red():
    led_ctrl.set_bottom_led(rm_define.armor_bottom_all, 255, 0, 0, rm_define.effect_flash)
    led_ctrl.set_top_led(rm_define.armor_top_all, 255, 0, 0, rm_define.effect_flash)
    led_ctrl.set_flash(rm_define.armor_all, FLASH_HZ)

# =============================================================================
# VISION
# =============================================================================
def people_get_first():
    info = vision_ctrl.get_people_detection_info()
    n = None
    x = 0.5
    y = 0.5
    w = 0.0
    h = 0.0
    try:
        n = info[0]
        x = info[1]
        y = info[2]
        w = info[3]
        h = info[4]
    except Exception:
        try:
            n = info[1]
            x = info[2]
            y = info[3]
            w = info[4]
            h = info[5]
        except Exception:
            return False, 0.5, 0.5, 0.0, 0.0
    if n is None:
        return False, 0.5, 0.5, 0.0, 0.0
    try:
        ni = int(n)
    except Exception:
        return False, 0.5, 0.5, 0.0, 0.0
    if ni < 1:
        return False, 0.5, 0.5, 0.0, 0.0
    try:
        if w < PERSON_MIN_W or h < PERSON_MIN_H:
            return False, x, y, w, h
    except Exception:
        return False, 0.5, 0.5, 0.0, 0.0
    if x < 0.0 or x > 1.0 or y < 0.0 or y > 1.0:
        return False, x, y, w, h
    return True, x, y, w, h

def people_seen():
    ok, x, y, w, h = people_get_first()
    return ok

def person_track_update():
    global g_person_miss, g_no_person_t0
    ok, x, y, w, h = people_get_first()
    if ok:
        g_person_miss = 0
        g_no_person_t0 = now_s()
        return True
    g_person_miss = g_person_miss + 1
    gimbal_stop()
    return False

def person_confirmed_lost():
    if g_person_miss < PERSON_MISS_NEED:
        return False
    if (now_s() - g_no_person_t0) < T_CLEAR:
        return False
    return True

def line_info_raw():
    return vision_ctrl.get_line_detection_info()

def line_raw_seen():
    if FORCE_NO_LINE:
        return False
    info = line_info_raw()
    try:
        n = len(info)
    except Exception:
        return False
    if n < 3:
        return False
    try:
        if info[2] is not None and info[2] >= 1:
            return True
    except Exception:
        return False
    try:
        if n >= 20:
            cx = info[19]
            if cx is not None and cx > 0.0 and cx < 1.0:
                return True
    except Exception:
        pass
    return False

def line_update():
    """多帧确认：更新 hit/miss 计数。"""
    global g_line_hit, g_line_miss
    if line_raw_seen():
        g_line_hit = g_line_hit + 1
        g_line_miss = 0
    else:
        g_line_miss = g_line_miss + 1
        g_line_hit = 0

def line_stable_true():
    return g_line_hit >= LINE_CONFIRM_FRAMES

def line_stable_false():
    return g_line_miss >= LINE_CONFIRM_FRAMES

def log_heartbeat():
    global g_last_hb_t
    if LOG_HEARTBEAT_S <= 0:
        return
    t = now_s()
    if g_last_hb_t > 0 and (t - g_last_hb_t) < LOG_HEARTBEAT_S:
        return
    g_last_hb_t = t
    has_p = False
    px = 0.0
    py = 0.0
    try:
        ok, px, py, w, h = people_get_first()
        has_p = ok
    except Exception:
        has_p = False
    extra = ""
    if g_state == STATE_PATROL:
        age = 0.0
        if g_patrol_line_t0 > 0:
            age = t - g_patrol_line_t0
        extra = " lineHit=%d miss=%d follow=%.1f" % (g_line_hit, g_line_miss, age)
    elif g_state == STATE_SCAN or g_state == STATE_LOST_SCAN:
        extra = " qi=%d/%d dir=%d yaw=%.0f lim=%.0f stuck=%d person=%s" % (
            g_scan_qi, len(g_scan_queue), g_scan_dir, get_yaw(), YAW_LIM, g_scan_stuck, str(has_p)
        )
    elif g_state == STATE_LOCK or g_state == STATE_FIRE:
        extra = " person=%s miss=%d xy=(%.2f,%.2f) phase=%d ir=%s" % (
            str(has_p), g_person_miss, px, py, g_fire_phase, str(g_ir_done)
        )
    else:
        extra = " lineHit=%d person=%s" % (g_line_hit, str(has_p))
    log("HB" + extra)

# =============================================================================
# ACTUATORS
# =============================================================================
def gimbal_set_pitch_line():
    gimbal_ctrl.pitch_ctrl(PITCH_LINE)

def gimbal_set_pitch_scan():
    gimbal_ctrl.pitch_ctrl(PITCH_SCAN)

def gimbal_stop():
    gimbal_ctrl.rotate_with_speed(0, 0)
    gimbal_ctrl.stop()

def chassis_halt():
    chassis_ctrl.stop()

def aim_pid_towards_person(dt):
    global g_iy, g_ey_prev, g_ip, g_ep_prev
    ok, x, y, w, h = people_get_first()
    if ok == False:
        gimbal_stop()
        pid_reset_aim()
        return False, False
    err_yaw = x - 0.5
    err_pitch = y - 0.5
    yaw_spd, g_iy, g_ey_prev = pid_step(
        err_yaw, g_iy, g_ey_prev, AIM_YAW_KP, AIM_YAW_KI, AIM_YAW_KD, AIM_YAW_OUT_MAX, dt
    )
    pitch_spd, g_ip, g_ep_prev = pid_step(
        err_pitch, g_ip, g_ep_prev, AIM_PITCH_KP, AIM_PITCH_KI, AIM_PITCH_KD, AIM_PITCH_OUT_MAX, dt
    )
    if abs(err_yaw) < AIM_OK_ERR and abs(err_pitch) < AIM_OK_ERR:
        gimbal_stop()
        return True, True
    gimbal_ctrl.rotate_with_speed(yaw_spd, -pitch_spd)
    return False, True

def line_follow_step():
    info = line_info_raw()
    vx = LINE_SPEED
    yaw_rate = 0.0
    try:
        n = len(info)
        if n >= 20:
            cx = info[19]
            err = cx - 0.5
            if abs(err) < 0.04:
                yaw_rate = 0.0
            else:
                yaw_rate = clamp(-err * LINE_PID_KP, -LINE_PID_OUT_MAX, LINE_PID_OUT_MAX)
    except Exception:
        yaw_rate = 0.0
    chassis_ctrl.move_with_speed(vx, 0, yaw_rate)

def fire_stop():
    gun_ctrl.stop()

def fire_ir_warn_once():
    global g_fire_count, g_ir_done
    if g_ir_done:
        log("IR_WARN skip already done")
        return
    if ENABLE_FIRE == False:
        log("IR_WARN skip ENABLE_FIRE=0")
        g_ir_done = True
        return
    gun_ctrl.set_fire_count(1)
    gun_ctrl.fire_once()
    g_fire_count = g_fire_count + 1
    g_ir_done = True
    log("IR_WARN fire_once count=%d" % g_fire_count)

def fire_bead_burst_start():
    if ENABLE_FIRE == False:
        log("BURST_ON skip ENABLE_FIRE=0")
        return
    gun_ctrl.set_fire_count(1)
    gun_ctrl.fire_continuous()
    log("BURST_ON continuous 1s")

def fire_bead_burst_stop():
    gun_ctrl.stop()
    log("BURST_OFF wait 1s")

# =============================================================================
# SCAN：扫到软限位（非物理 360°，因 yaw≈±250）
# 队列元素仅为 dir: +1 扫到 +YAW_LIM，-1 扫到 -YAW_LIM
# =============================================================================
def scan_queue_full_cw_ccw():
    return [1, -1]

def scan_save_breakpoint():
    global g_brk_valid, g_brk_dir
    g_brk_valid = True
    g_brk_dir = g_scan_dir
    log("BRK save dir=%d yaw=%.0f" % (g_brk_dir, get_yaw()))

def scan_queue_after_lost():
    if g_brk_valid == False:
        log("LOST queue: full +lim then -lim")
        return [1, -1]
    if g_brk_dir > 0:
        log("LOST queue: finish +lim then -lim")
        return [1, -1]
    log("LOST queue: finish -lim then full +lim/-lim")
    return [-1, 1, -1]

def scan_at_limit(dir_s):
    yaw = get_yaw()
    if dir_s > 0:
        return yaw >= (YAW_LIM - YAW_ARRIVE)
    return yaw <= (-YAW_LIM + YAW_ARRIVE)

def scan_load_segment(qi):
    global g_scan_qi, g_scan_dir, g_scan_last_yaw, g_scan_stuck
    g_scan_qi = qi
    g_scan_dir = g_scan_queue[qi]
    g_scan_last_yaw = get_yaw()
    g_scan_stuck = 0
    if g_scan_dir > 0:
        log("SCAN seg qi=%d dir=+1 to +%.0f yaw0=%.0f" % (qi, YAW_LIM, g_scan_last_yaw))
    else:
        log("SCAN seg qi=%d dir=-1 to -%.0f yaw0=%.0f" % (qi, YAW_LIM, g_scan_last_yaw))

def scan_start_queue(queue, reason):
    global g_scan_queue
    g_scan_queue = queue
    gimbal_set_pitch_scan()
    time.sleep(0.05)
    if len(g_scan_queue) <= 0:
        log("SCAN empty queue?")
        return
    scan_load_segment(0)
    log("SCAN queue start n=%d | %s" % (len(g_scan_queue), reason))

def scan_tick_turn():
    """
    向软限位转动；到位或卡住则返回 True。
    """
    global g_scan_last_yaw, g_scan_stuck
    yaw = get_yaw()
    if scan_at_limit(g_scan_dir):
        log("SCAN hit lim yaw=%.0f dir=%d" % (yaw, g_scan_dir))
        return True
    d = yaw - g_scan_last_yaw
    if abs(d) < 0.4:
        g_scan_stuck = g_scan_stuck + 1
    else:
        g_scan_stuck = 0
    g_scan_last_yaw = yaw
    if g_scan_stuck >= SCAN_STUCK_FRAMES:
        log("SCAN stuck at yaw=%.0f treat as lim dir=%d" % (yaw, g_scan_dir))
        return True
    spd = SCAN_YAW_SPEED
    if spd < 1.0:
        spd = 1.0
    gimbal_ctrl.rotate_with_speed(g_scan_dir * spd, 0)
    return False

def scan_advance_or_finish():
    global g_scan_qi
    gimbal_stop()
    nq = len(g_scan_queue)
    ni = g_scan_qi + 1
    if ni < nq:
        scan_load_segment(ni)
        return "next"
    return "done"

def scan_finish_recenter():
    gimbal_stop()
    log("SCAN recenter after full queue")
    gimbal_ctrl.recenter()
    time.sleep(0.25)
    gimbal_set_pitch_line()
# =============================================================================
# STATE MACHINE
# =============================================================================
def set_state(s, reason):
    global g_state, g_state_t0, g_no_person_t0, g_person_miss
    global g_patrol_line_t0, g_fire_count, g_fire_phase, g_phase_t0
    global g_ir_done, g_line_hit, g_line_miss, g_brk_valid
    old = g_state
    g_state = s
    g_state_t0 = now_s()
    # 注意：进 FIRE 不重置 g_ir_done（避免二次示警）
    if s != STATE_FIRE:
        g_no_person_t0 = now_s()
        g_person_miss = 0
    log("STATE %s -> %s | %s" % (state_name(old), state_name(s), reason))

    if s == STATE_PATROL:
        fire_stop()
        chassis_halt()
        gimbal_stop()
        robot_ctrl.set_mode(rm_define.robot_mode_free)
        gimbal_set_pitch_line()
        leds_normal()
        g_patrol_line_t0 = 0.0
        g_line_hit = 0
        g_line_miss = 0
        vision_ctrl.enable_detection(rm_define.vision_detection_line)

    if s == STATE_SCAN:
        fire_stop()
        chassis_halt()
        robot_ctrl.set_mode(rm_define.robot_mode_free)
        leds_normal()
        g_brk_valid = False
        scan_start_queue(scan_queue_full_cw_ccw(), "normal_scan")

    if s == STATE_LOST_SCAN:
        fire_stop()
        chassis_halt()
        gimbal_stop()
        pid_reset_aim()
        robot_ctrl.set_mode(rm_define.robot_mode_free)
        leds_normal()
        q = scan_queue_after_lost()
        scan_start_queue(q, "lost_scan")

    if s == STATE_LOCK:
        # 从 SCAN/LOST_SCAN 来则保存断点
        if old == STATE_SCAN or old == STATE_LOST_SCAN:
            scan_save_breakpoint()
        fire_stop()
        chassis_halt()
        robot_ctrl.set_mode(rm_define.robot_mode_free)
        gimbal_set_pitch_scan()
        pid_reset_aim()
        leds_alert_red()
        g_fire_count = 0
        g_ir_done = False
        g_fire_phase = FIRE_PHASE_AIM
        g_phase_t0 = now_s()
        g_person_miss = 0
        g_no_person_t0 = now_s()
        ok, x, y, w, h = people_get_first()
        log("LOCK ok=%s xy=(%.2f,%.2f)" % (str(ok), x, y))

    if s == STATE_FIRE:
        chassis_halt()
        pid_reset_aim()
        leds_alert_red()
        # 保留 g_ir_done
        if g_ir_done:
            g_fire_phase = FIRE_PHASE_IR_DONE
        else:
            g_fire_phase = FIRE_PHASE_AIM
        g_phase_t0 = now_s()
        g_person_miss = 0
        g_no_person_t0 = now_s()
        log("FIRE enter ir_done=%s" % str(g_ir_done))

    if s == STATE_RECOVER:
        fire_stop()
        chassis_halt()
        gimbal_stop()
        pid_reset_aim()
        gimbal_set_pitch_line()
        leds_normal()
        g_line_hit = 0
        g_line_miss = 0
        g_brk_valid = False
        log("RECOVER find line")

def tick_patrol():
    global g_patrol_line_t0
    line_update()
    if people_seen():
        log("PATROL person -> LOCK")
        set_state(STATE_LOCK, "person_on_patrol")
        return
    # 多帧确认无线才去 SCAN
    if line_stable_false():
        g_patrol_line_t0 = 0.0
        chassis_halt()
        set_state(STATE_SCAN, "no_line_stable")
        return
    # 尚未确认有线：先别当循线成功
    if line_stable_true() == False:
        chassis_halt()
        return
    if g_patrol_line_t0 <= 0.0:
        g_patrol_line_t0 = now_s()
        log("PATROL line stable follow %.1fs" % T_MOVE)
    line_follow_step()
    if (now_s() - g_patrol_line_t0) >= T_MOVE:
        g_patrol_line_t0 = 0.0
        set_state(STATE_SCAN, "follow_time_up")

def tick_scan_common(is_lost):
    chassis_halt()
    # 允许中途 LOCK
    if people_seen():
        gimbal_stop()
        log("SCAN person -> LOCK")
        set_state(STATE_LOCK, "person_on_scan")
        return
    done_seg = scan_tick_turn()
    if done_seg == False:
        return
    # 当前段完成
    log("SCAN seg done yaw=%.0f dir=%d" % (get_yaw(), g_scan_dir))
    adv = scan_advance_or_finish()
    if adv == "next":
        return
    # 整队完成
    scan_finish_recenter()
    if is_lost:
        log("LOST_SCAN queue done -> RECOVER")
        set_state(STATE_RECOVER, "lost_scan_done")
        return
    # 普通 SCAN：多帧看线
    line_update()
    # 给几帧采样
    i = 0
    while i < LINE_CONFIRM_FRAMES:
        line_update()
        time.sleep(LOOP_DT)
        i = i + 1
    if line_stable_true():
        log("SCAN done line stable -> PATROL")
        set_state(STATE_PATROL, "scan_has_line")
        return
    log("SCAN done no line stable -> SCAN again")
    set_state(STATE_SCAN, "rescan_no_line")

def tick_scan():
    tick_scan_common(False)

def tick_lost_scan():
    tick_scan_common(True)

def tick_lock():
    global g_fire_phase, g_phase_t0
    chassis_halt()
    has = person_track_update()
    if has:
        aim_pid_towards_person(LOOP_DT)
        if g_ir_done == False and state_age() >= T_AIM_BEFORE_IR:
            fire_ir_warn_once()
            g_fire_phase = FIRE_PHASE_IR_DONE
            g_phase_t0 = now_s()
            log("LOCK IR warn -> FIRE")
            set_state(STATE_FIRE, "after_ir_warn")
        return
    if person_confirmed_lost():
        log("LOCK lost miss=%d -> LOST_SCAN" % g_person_miss)
        set_state(STATE_LOST_SCAN, "person_lost")

def tick_fire():
    global g_fire_phase, g_phase_t0
    chassis_halt()
    has = person_track_update()
    if has == False:
        fire_stop()
        if person_confirmed_lost():
            log("FIRE lost miss=%d -> LOST_SCAN" % g_person_miss)
            set_state(STATE_LOST_SCAN, "person_lost_fire")
        return

    aim_pid_towards_person(LOOP_DT)

    # 仅一次示警
    if g_ir_done == False:
        if state_age() >= T_AIM_BEFORE_IR:
            fire_ir_warn_once()
            g_fire_phase = FIRE_PHASE_BURST_ON
            g_phase_t0 = now_s()
            fire_bead_burst_start()
            log("FIRE burst start phase_age reset")
        return

    if g_fire_phase == FIRE_PHASE_IR_DONE:
        g_fire_phase = FIRE_PHASE_BURST_ON
        g_phase_t0 = now_s()
        fire_bead_burst_start()
        return

    if g_fire_phase == FIRE_PHASE_BURST_ON:
        if phase_age() >= T_BURST_ON:
            fire_bead_burst_stop()
            g_fire_phase = FIRE_PHASE_BURST_OFF
            g_phase_t0 = now_s()
            log("FIRE phase OFF age was >=%.1f" % T_BURST_ON)
        return

    if g_fire_phase == FIRE_PHASE_BURST_OFF:
        if phase_age() >= T_BURST_OFF:
            g_fire_phase = FIRE_PHASE_BURST_ON
            g_phase_t0 = now_s()
            fire_bead_burst_start()
            log("FIRE phase ON again")
        return

def tick_recover():
    fire_stop()
    chassis_halt()
    gimbal_stop()
    gimbal_set_pitch_line()
    line_update()
    if line_stable_true():
        log("RECOVER line stable -> PATROL")
        set_state(STATE_PATROL, "line_found")
        return
    if line_stable_false() and state_age() >= 0.6:
        log("RECOVER no line stable -> SCAN")
        set_state(STATE_SCAN, "still_no_line")
        return

# =============================================================================
# ENTRY
# =============================================================================
def setup():
    log("setup begin")
    robot_ctrl.set_mode(rm_define.robot_mode_free)
    chassis_halt()
    gimbal_ctrl.recenter()
    time.sleep(0.3)
    vision_ctrl.enable_detection(rm_define.vision_detection_people)
    vision_ctrl.enable_detection(rm_define.vision_detection_line)
    vision_ctrl.line_follow_color_set(rm_define.line_follow_color_blue)
    media_ctrl.exposure_value_update(rm_define.exposure_value_medium)
    gun_ctrl.set_fire_count(1)
    leds_normal()
    gimbal_set_pitch_line()
    pid_reset_aim()
    log("setup done v1.4.0 yaw-scan + line confirm + single IR")

def start():
    global g_state
    print("======== Line Guard start ========")
    print("# LINE_GUARD_VERSION=1.5.0 stamp=2026-08-02 11:45:06")
    log("program start")
    setup()
    set_state(STATE_PATROL, "boot")
    while True:
        if g_state == STATE_PATROL:
            tick_patrol()
        elif g_state == STATE_SCAN:
            tick_scan()
        elif g_state == STATE_LOST_SCAN:
            tick_lost_scan()
        elif g_state == STATE_LOCK:
            tick_lock()
        elif g_state == STATE_FIRE:
            tick_fire()
        elif g_state == STATE_RECOVER:
            tick_recover()
        else:
            log("bad state")
            set_state(STATE_PATROL, "bad_state")
        log_heartbeat()
        time.sleep(LOOP_DT)
