# LINE_GUARD_VERSION=1.2.0 stamp=2026-08-01 12:23:17  (paste this whole file; check stamp matches latest)
# -*- coding: utf-8 -*-
# S1 Line Guard v1.2 — 单文件，整段粘贴进 App 实验室 Python
#
# PATROL: 有蓝线则循线 T_MOVE 秒后 SCAN；无蓝线则进入 SCAN（不空转找线）
# SCAN: 云台匀速转满 N 圈；见人->LOCK
#       转完仍无线：云台回中/回起点，继续 SCAN（禁止 PATROL<->SCAN 死循环）
#       转完有线：回 PATROL
# LOCK: 红闪 + PID 瞄准；先红外/光电示警一发，再进入水弹连发节奏
# FIRE: 水弹连发 1 秒 -> 停 1 秒 -> 再连发… 直到人离开
# 人离开: 至少再 SCAN 满 1 圈，再进入找线(PATROL)
#
# 说明：实验室 gun API 通常只有 fire_once/fire_continuous/stop。
# 红外 vs 水弹若 App 设置里可切换，示警前请确保能打红外；代码里会先 fire_once 作示警，
# 再 fire_continuous 作水弹连发（若模式由 App 全局设置，请先在设置里选好水弹再测连发）。
#
# 粘贴：App 内全选删除后再整文件粘贴。控制台看 stamp= 是否最新。

# =============================================================================
# CONFIG
# =============================================================================
T_MOVE = 3.0
T_CLEAR = 2.0
LOOP_DT = 0.05
LOG_HEARTBEAT_S = 1.0

PITCH_LINE = -20
PITCH_SCAN = 5
SCAN_YAW_SPEED = 180.0
SCAN_TURNS = 1
# 人离开后强制至少再扫几圈再找线
SCAN_TURNS_AFTER_LOST = 1

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

# 射击节奏
T_AIM_BEFORE_IR = 1.0     # 锁定后先 PID 瞄准再红外示警（秒）
T_BURST_ON = 1.0          # 水弹连发持续（秒）
T_BURST_OFF = 1.0         # 连发后等待（秒）

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

# FIRE 子阶段
FIRE_PHASE_AIM = 0
FIRE_PHASE_IR_DONE = 1
FIRE_PHASE_BURST_ON = 2
FIRE_PHASE_BURST_OFF = 3

g_state = STATE_INIT
g_state_t0 = 0.0
g_no_person_t0 = 0.0
g_patrol_line_t0 = 0.0
g_scan_deg_done = 0.0
g_scan_need_deg = 360.0
g_last_hb_t = 0.0
g_fire_count = 0
g_fire_phase = FIRE_PHASE_AIM
g_phase_t0 = 0.0
g_ir_done = False
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

# =============================================================================
# VISION
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
        extra = " line=%s follow=%.1f/%.1f" % (str(has_line), age, T_MOVE)
    elif g_state == STATE_SCAN or g_state == STATE_LOST_SCAN:
        pct = 0.0
        if g_scan_need_deg > 0:
            pct = 100.0 * g_scan_deg_done / g_scan_need_deg
        extra = " deg=%.0f/%.0f pct=%.0f person=%s line=%s" % (
            g_scan_deg_done, g_scan_need_deg, pct, str(has_p), str(has_line)
        )
    elif g_state == STATE_LOCK or g_state == STATE_FIRE:
        extra = " person=%s xy=(%.2f,%.2f) phase=%d fires=%d" % (
            str(has_p), px, py, g_fire_phase, g_fire_count
        )
    else:
        extra = " line=%s person=%s" % (str(has_line), str(has_p))
    log("HB" + extra)

# =============================================================================
# ACTUATORS
# =============================================================================
def gimbal_set_pitch_line():
    gimbal_ctrl.pitch_ctrl(PITCH_LINE)

def gimbal_set_pitch_scan():
    gimbal_ctrl.pitch_ctrl(PITCH_SCAN)

def gimbal_stop():
    gimbal_ctrl.stop()

def gimbal_scan_reset_pose():
    """扫完一圈回起点：回中再抬到扫人俯仰。"""
    gimbal_stop()
    gimbal_ctrl.recenter()
    time.sleep(0.15)
    gimbal_set_pitch_scan()

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
    yaw_spd, g_iy, g_ey_prev = pid_step(
        err_yaw, g_iy, g_ey_prev, AIM_YAW_KP, AIM_YAW_KI, AIM_YAW_KD, AIM_YAW_OUT_MAX, dt
    )
    pitch_spd, g_ip, g_ep_prev = pid_step(
        err_pitch, g_ip, g_ep_prev, AIM_PITCH_KP, AIM_PITCH_KI, AIM_PITCH_KD, AIM_PITCH_OUT_MAX, dt
    )
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

def fire_stop():
    gun_ctrl.stop()

def fire_ir_warn_once():
    """光电/红外示警一发（实验室无独立模式 API 时等同 fire_once；请在 App 设置配合）。"""
    global g_fire_count
    if ENABLE_FIRE == False:
        log("IR_WARN skip ENABLE_FIRE=0")
        return
    gun_ctrl.set_fire_count(1)
    gun_ctrl.fire_once()
    g_fire_count = g_fire_count + 1
    log("IR_WARN fire_once count=%d" % g_fire_count)

def fire_bead_burst_start():
    """水弹连发开始。"""
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
# STATE MACHINE
# =============================================================================
def set_state(s, reason):
    global g_state, g_state_t0, g_no_person_t0
    global g_patrol_line_t0, g_scan_deg_done, g_scan_need_deg
    global g_fire_count, g_fire_phase, g_phase_t0, g_ir_done
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
        vision_ctrl.enable_detection(rm_define.vision_detection_line)

    if s == STATE_SCAN:
        fire_stop()
        chassis_halt()
        robot_ctrl.set_mode(rm_define.robot_mode_free)
        gimbal_scan_reset_pose()
        g_scan_deg_done = 0.0
        g_scan_need_deg = 360.0 * SCAN_TURNS
        leds_normal()
        log("SCAN need=%.0f speed=%.0f" % (g_scan_need_deg, SCAN_YAW_SPEED))

    if s == STATE_LOST_SCAN:
        fire_stop()
        chassis_halt()
        robot_ctrl.set_mode(rm_define.robot_mode_free)
        gimbal_scan_reset_pose()
        g_scan_deg_done = 0.0
        g_scan_need_deg = 360.0 * SCAN_TURNS_AFTER_LOST
        leds_normal()
        log("LOST_SCAN need=%.0f (then find line)" % g_scan_need_deg)

    if s == STATE_LOCK:
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
        ok, x, y, w, h = people_get_first()
        log("LOCK ok=%s xy=(%.2f,%.2f)" % (str(ok), x, y))

    if s == STATE_FIRE:
        chassis_halt()
        pid_reset_aim()
        leds_alert_red()
        g_fire_phase = FIRE_PHASE_AIM
        g_phase_t0 = now_s()
        g_ir_done = False
        log("FIRE enter aim then IR then burst cycle")

    if s == STATE_RECOVER:
        fire_stop()
        chassis_halt()
        gimbal_stop()
        gimbal_set_pitch_line()
        leds_normal()
        log("RECOVER -> try find line")

def tick_patrol():
    global g_patrol_line_t0
    if people_seen():
        log("PATROL person -> LOCK")
        set_state(STATE_LOCK, "person_on_patrol")
        return
    # 无线：直接 SCAN，不再在 PATROL 空转
    if line_seen() == False:
        g_patrol_line_t0 = 0.0
        chassis_halt()
        set_state(STATE_SCAN, "no_line")
        return
    if g_patrol_line_t0 <= 0.0:
        g_patrol_line_t0 = now_s()
        log("PATROL line ok follow %.1fs" % T_MOVE)
    line_follow_step()
    if (now_s() - g_patrol_line_t0) >= T_MOVE:
        g_patrol_line_t0 = 0.0
        set_state(STATE_SCAN, "follow_time_up")

def tick_scan_common(after_lost):
    """SCAN 与 LOST_SCAN 共用转圈逻辑。"""
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
    if g_scan_deg_done < g_scan_need_deg:
        return
    # 一圈/N 圈结束
    gimbal_stop()
    if after_lost:
        # 人离开后扫够圈 -> 找线
        log("LOST_SCAN done -> RECOVER find line")
        set_state(STATE_RECOVER, "lost_scan_done")
        return
    # 普通 SCAN 结束
    if line_seen():
        log("SCAN done has line -> PATROL")
        set_state(STATE_PATROL, "scan_has_line")
        return
    # 仍无线：回起点继续扫，绝不 PATROL 空转
    log("SCAN done no line -> recenter and SCAN again")
    gimbal_scan_reset_pose()
    g_scan_deg_done = 0.0
    g_scan_need_deg = 360.0 * SCAN_TURNS
    log("SCAN restart need=%.0f" % g_scan_need_deg)

def tick_scan():
    tick_scan_common(False)

def tick_lost_scan():
    tick_scan_common(True)

def tick_lock():
    global g_no_person_t0, g_ir_done, g_fire_phase, g_phase_t0
    chassis_halt()
    aligned, has = aim_pid_towards_person(LOOP_DT)
    if has == False:
        lost = now_s() - g_no_person_t0
        if lost >= T_CLEAR:
            log("LOCK lost -> LOST_SCAN")
            set_state(STATE_LOST_SCAN, "person_lost")
        return
    g_no_person_t0 = now_s()
    # 先瞄准一段时间，再红外示警，然后进 FIRE 连发节奏
    if g_ir_done == False and state_age() >= T_AIM_BEFORE_IR:
        fire_ir_warn_once()
        g_ir_done = True
        g_fire_phase = FIRE_PHASE_IR_DONE
        g_phase_t0 = now_s()
        log("LOCK IR warn done -> FIRE burst cycle")
        set_state(STATE_FIRE, "after_ir_warn")

def tick_fire():
    global g_no_person_t0, g_fire_phase, g_phase_t0, g_ir_done
    chassis_halt()
    aligned, has = aim_pid_towards_person(LOOP_DT)
    if has == False:
        lost_t = now_s() - g_no_person_t0
        if lost_t >= T_CLEAR:
            fire_stop()
            log("FIRE lost -> LOST_SCAN")
            set_state(STATE_LOST_SCAN, "person_lost_fire")
        return
    g_no_person_t0 = now_s()

    # 若从别处进 FIRE 尚未红外示警，先补一发
    if g_ir_done == False:
        if state_age() >= T_AIM_BEFORE_IR:
            fire_ir_warn_once()
            g_ir_done = True
            g_fire_phase = FIRE_PHASE_BURST_ON
            g_phase_t0 = now_s()
            fire_bead_burst_start()
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
        return

    if g_fire_phase == FIRE_PHASE_BURST_OFF:
        if phase_age() >= T_BURST_OFF:
            # 人还在：再连发 1 秒
            g_fire_phase = FIRE_PHASE_BURST_ON
            g_phase_t0 = now_s()
            fire_bead_burst_start()
        return

def tick_recover():
    """找线：有线则 PATROL；无线则再次原地 SCAN（回中后扫）。"""
    fire_stop()
    chassis_halt()
    gimbal_set_pitch_line()
    if line_seen():
        log("RECOVER found line -> PATROL")
        set_state(STATE_PATROL, "line_found")
        return
    if state_age() >= 0.5:
        log("RECOVER still no line -> SCAN")
        set_state(STATE_SCAN, "still_no_line")

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
    log("setup done v1.2.0 move=%.1f scan_spd=%.0f fire=%s" % (T_MOVE, SCAN_YAW_SPEED, str(ENABLE_FIRE)))

def start():
    global g_state
    print("======== Line Guard start ========")
    print("# LINE_GUARD_VERSION=1.2.0 stamp=2026-08-01 12:23:17")
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
