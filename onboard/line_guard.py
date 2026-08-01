# -*- coding: utf-8 -*-
# S1 Line Guard v1.1 — 单文件，整段粘贴进 App 实验室 Python
# PATROL: 有线循线 3s；无线原地 SCAN
# SCAN: 云台匀速转满 N 圈扫人
# LOCK/FIRE: 红闪 + PID 瞄准；满 3s 后每 1s 点射
# 调试：print 输出到控制台；截图请放入项目 logs/
# 粘贴：App 内全选删除后再粘贴，勿混旧代码

# =============================================================================
# CONFIG
# =============================================================================
T_MOVE = 3.0
T_WARN_BEFORE_FIRE = 3.0
T_FIRE_INTERVAL = 1.0
T_CLEAR = 2.0
LOOP_DT = 0.05
LOG_HEARTBEAT_S = 1.0

PITCH_LINE = -20
PITCH_SCAN = 5
SCAN_YAW_SPEED = 180.0
SCAN_TURNS = 1

LINE_SPEED = 0.35
LINE_PID_KP = 80.0
LINE_PID_OUT_MAX = 80.0

AIM_YAW_KP = 180.0
AIM_YAW_KI = 0.0
AIM_YAW_KD = 8.0
AIM_YAW_OUT_MAX = 120.0
AIM_PITCH_KP = 120.0
AIM_PITCH_KI = 0.0
AIM_PITCH_KD = 6.0
AIM_PITCH_OUT_MAX = 80.0
AIM_OK_ERR = 0.08

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
STATE_RECOVER = 5

g_state = STATE_INIT
g_state_t0 = 0.0
g_no_person_t0 = 0.0
g_last_fire_t = 0.0
g_patrol_line_t0 = 0.0
g_scan_deg_done = 0.0
g_last_hb_t = 0.0
g_fire_count = 0
g_iy = 0.0
g_ey_prev = 0.0
g_ip = 0.0
g_ep_prev = 0.0

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

def scan_degrees_needed():
    return 360.0 * SCAN_TURNS

def pid_reset_aim():
    global g_iy, g_ey_prev, g_ip, g_ep_prev
    g_iy = 0.0
    g_ey_prev = 0.0
    g_ip = 0.0
    g_ep_prev = 0.0

def pid_step(err, i_acc, e_prev, kp, ki, kd, out_max, dt):
    i_new = i_acc + err * dt
    i_new = clamp(i_new, -2.0, 2.0)
    if dt > 0.0001:
        d = (err - e_prev) / dt
    else:
        d = 0.0
    out = kp * err + ki * i_new + kd * d
    out = clamp(out, -out_max, out_max)
    return out, i_new, err

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

def leds_off():
    led_ctrl.turn_off(rm_define.armor_all)

# =============================================================================
# VISION: people / line
# =============================================================================
def people_get_first():
    info = vision_ctrl.get_people_detection_info()
    try:
        n = info[0]
    except Exception:
        return False, 0.5, 0.5, 0.0, 0.0
    if n is None:
        return False, 0.5, 0.5, 0.0, 0.0
    if n < 1:
        return False, 0.5, 0.5, 0.0, 0.0
    try:
        x = info[1]
        y = info[2]
        w = info[3]
        h = info[4]
    except Exception:
        return False, 0.5, 0.5, 0.0, 0.0
    return True, x, y, w, h

def people_seen():
    ok, x, y, w, h = people_get_first()
    return ok

def line_info_raw():
    return vision_ctrl.get_line_detection_info()

def line_seen():
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

def log_heartbeat():
    global g_last_hb_t
    if LOG_HEARTBEAT_S <= 0:
        return
    t = now_s()
    if g_last_hb_t > 0 and (t - g_last_hb_t) < LOG_HEARTBEAT_S:
        return
    g_last_hb_t = t
    has_line = False
    has_p = False
    px = 0.0
    py = 0.0
    try:
        has_line = line_seen()
    except Exception:
        has_line = False
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
        extra = " line=%s follow=%.1f/%.1fs" % (str(has_line), age, T_MOVE)
    elif g_state == STATE_SCAN:
        need = scan_degrees_needed()
        pct = 0.0
        if need > 0:
            pct = 100.0 * g_scan_deg_done / need
        extra = " deg=%.0f/%.0f pct=%.0f person=%s" % (g_scan_deg_done, need, pct, str(has_p))
    elif g_state == STATE_LOCK or g_state == STATE_FIRE:
        extra = " person=%s xy=(%.2f,%.2f) age=%.1f fires=%d" % (str(has_p), px, py, state_age(), g_fire_count)
    else:
        extra = " line=%s person=%s age=%.1f" % (str(has_line), str(has_p), state_age())
    log("HB" + extra)

# =============================================================================
# GIMBAL / CHASSIS / FIRE
# =============================================================================
def gimbal_set_pitch_line():
    gimbal_ctrl.pitch_ctrl(PITCH_LINE)

def gimbal_set_pitch_scan():
    gimbal_ctrl.pitch_ctrl(PITCH_SCAN)

def gimbal_stop():
    gimbal_ctrl.stop()

def chassis_halt():
    chassis_ctrl.stop()

def aim_pid_towards_person(dt):
    global g_iy, g_ey_prev, g_ip, g_ep_prev
    ok, x, y, w, h = people_get_first()
    if ok == False:
        gimbal_stop()
        return False, False
    err_yaw = x - 0.5
    err_pitch = y - 0.5
    yaw_spd, g_iy, g_ey_prev = pid_step(err_yaw, g_iy, g_ey_prev, AIM_YAW_KP, AIM_YAW_KI, AIM_YAW_KD, AIM_YAW_OUT_MAX, dt)
    pitch_spd, g_ip, g_ep_prev = pid_step(err_pitch, g_ip, g_ep_prev, AIM_PITCH_KP, AIM_PITCH_KI, AIM_PITCH_KD, AIM_PITCH_OUT_MAX, dt)
    gimbal_ctrl.rotate_with_speed(yaw_spd, -pitch_spd)
    aligned = False
    if abs(err_yaw) < AIM_OK_ERR and abs(err_pitch) < AIM_OK_ERR:
        aligned = True
    return aligned, True

def line_follow_step():
    info = line_info_raw()
    vx = LINE_SPEED
    yaw_rate = 0.0
    try:
        n = len(info)
        if n >= 20:
            cx = info[19]
            err = cx - 0.5
            yaw_rate = clamp(-err * LINE_PID_KP, -LINE_PID_OUT_MAX, LINE_PID_OUT_MAX)
    except Exception:
        yaw_rate = 0.0
    chassis_ctrl.move_with_speed(vx, 0, yaw_rate)

def fire_once_safe():
    global g_fire_count
    if ENABLE_FIRE:
        gun_ctrl.set_fire_count(1)
        gun_ctrl.fire_once()
        g_fire_count = g_fire_count + 1
        log("FIRE_ONCE count=%d" % g_fire_count)
    else:
        log("FIRE_SKIP ENABLE_FIRE=0")

def fire_stop():
    gun_ctrl.stop()

# =============================================================================
# STATE MACHINE
# =============================================================================
def set_state(s, reason):
    global g_state, g_state_t0, g_no_person_t0, g_last_fire_t
    global g_patrol_line_t0, g_scan_deg_done, g_fire_count
    old = g_state
    g_state = s
    g_state_t0 = now_s()
    g_no_person_t0 = now_s()
    log("STATE %s -> %s | %s" % (state_name(old), state_name(s), reason))
    if s == STATE_PATROL:
        fire_stop()
        chassis_halt()
        robot_ctrl.set_mode(rm_define.robot_mode_free)
        gimbal_set_pitch_line()
        leds_normal()
        g_patrol_line_t0 = 0.0
        g_fire_count = 0
        vision_ctrl.enable_detection(rm_define.vision_detection_line)
    if s == STATE_SCAN:
        fire_stop()
        chassis_halt()
        robot_ctrl.set_mode(rm_define.robot_mode_free)
        gimbal_set_pitch_scan()
        g_scan_deg_done = 0.0
        leds_normal()
        log("SCAN turns=%d speed=%.0f need=%.0f" % (SCAN_TURNS, SCAN_YAW_SPEED, scan_degrees_needed()))
    if s == STATE_LOCK:
        chassis_halt()
        robot_ctrl.set_mode(rm_define.robot_mode_free)
        gimbal_set_pitch_scan()
        pid_reset_aim()
        leds_alert_red()
        g_no_person_t0 = now_s()
        g_fire_count = 0
        ok, x, y, w, h = people_get_first()
        log("LOCK ok=%s xy=(%.2f,%.2f)" % (str(ok), x, y))
    if s == STATE_FIRE:
        chassis_halt()
        pid_reset_aim()
        leds_alert_red()
        g_last_fire_t = 0.0
        log("FIRE enter interval=%.1f" % T_FIRE_INTERVAL)
    if s == STATE_RECOVER:
        fire_stop()
        chassis_halt()
        gimbal_stop()
        gimbal_set_pitch_line()
        leds_normal()
        log("RECOVER")

def tick_patrol():
    global g_patrol_line_t0
    if people_seen():
        log("PATROL person -> LOCK")
        set_state(STATE_LOCK, "person_on_patrol")
        return
    if line_seen() == False:
        g_patrol_line_t0 = 0.0
        chassis_halt()
        set_state(STATE_SCAN, "no_line")
        return
    if g_patrol_line_t0 <= 0.0:
        g_patrol_line_t0 = now_s()
        log("PATROL line ok, follow %.1fs" % T_MOVE)
    line_follow_step()
    if (now_s() - g_patrol_line_t0) >= T_MOVE:
        g_patrol_line_t0 = 0.0
        set_state(STATE_SCAN, "follow_time_up")

def tick_scan():
    global g_scan_deg_done
    chassis_halt()
    if people_seen():
        gimbal_stop()
        log("SCAN person -> LOCK")
        set_state(STATE_LOCK, "person_on_scan")
        return
    spd = SCAN_YAW_SPEED
    if spd < 1.0:
        spd = 1.0
    g_scan_deg_done = g_scan_deg_done + spd * LOOP_DT
    gimbal_ctrl.rotate_with_speed(spd, 0)
    if g_scan_deg_done >= scan_degrees_needed():
        gimbal_stop()
        log("SCAN done -> PATROL")
        set_state(STATE_PATROL, "scan_complete")

def tick_lock():
    global g_no_person_t0
    chassis_halt()
    aligned, has = aim_pid_towards_person(LOOP_DT)
    if has == False:
        lost = now_s() - g_no_person_t0
        if lost >= T_CLEAR:
            log("LOCK lost person -> RECOVER")
            set_state(STATE_RECOVER, "person_lost")
        return
    g_no_person_t0 = now_s()
    if state_age() >= T_WARN_BEFORE_FIRE:
        log("LOCK time up aligned=%s -> FIRE" % str(aligned))
        set_state(STATE_FIRE, "warn_time_up")

def tick_fire():
    global g_no_person_t0, g_last_fire_t
    chassis_halt()
    aligned, has = aim_pid_towards_person(LOOP_DT)
    if has == False:
        lost_t = now_s() - g_no_person_t0
        if lost_t >= T_CLEAR:
            fire_stop()
            log("FIRE lost person -> RECOVER")
            set_state(STATE_RECOVER, "person_lost_fire")
        return
    g_no_person_t0 = now_s()
    t = now_s()
    need_fire = False
    if g_last_fire_t <= 0.0:
        need_fire = True
    if (t - g_last_fire_t) >= T_FIRE_INTERVAL:
        need_fire = True
    if need_fire:
        fire_once_safe()
        g_last_fire_t = t

def tick_recover():
    chassis_halt()
    gimbal_set_pitch_line()
    if state_age() >= 0.8:
        set_state(STATE_PATROL, "recover_done")

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
    log("setup done move=%.1f turns=%d spd=%.0f fire=%s" % (T_MOVE, SCAN_TURNS, SCAN_YAW_SPEED, str(ENABLE_FIRE)))

def start():
    global g_state
    print("======== Line Guard v1.1 start ========")
    log("program start")
    setup()
    set_state(STATE_PATROL, "boot")
    while True:
        if g_state == STATE_PATROL:
            tick_patrol()
        elif g_state == STATE_SCAN:
            tick_scan()
        elif g_state == STATE_LOCK:
            tick_lock()
        elif g_state == STATE_FIRE:
            tick_fire()
        elif g_state == STATE_RECOVER:
            tick_recover()
        else:
            log("bad state reset")
            set_state(STATE_PATROL, "bad_state")
        log_heartbeat()
        time.sleep(LOOP_DT)
