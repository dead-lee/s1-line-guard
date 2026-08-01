# LINE_GUARD_VERSION=1.3.0 stamp=2026-08-01 21:25:32  (paste this whole file; check stamp matches latest)
# -*- coding: utf-8 -*-
# S1 Line Guard v1.3 — 单文件，整段粘贴进 App 实验室 Python
#
# 相对 v1.2：
# - 行人丢失：连续 miss 帧 + 最短丢失时间，立刻停云台，避免“假检测”死锁抖动瞄准
# - PID：参考 DJI 实验室示例（P+D、I=0、限幅），加死区；对准后输出 0
# - 检测：要求 N>=1 且框宽高足够大，过滤噪声点
#
# PATROL / SCAN / LOCK / FIRE / LOST_SCAN / RECOVER 逻辑同前（无线持续 SCAN、示警+连发节奏）

# =============================================================================
# CONFIG
# =============================================================================
T_MOVE = 3.0
T_CLEAR = 0.8             # 确认离开的最短时间（秒）；配合 miss 帧
PERSON_MISS_NEED = 6      # 连续多少帧看不到人（约 6*0.05≈0.3s）才开始计离开
LOOP_DT = 0.05
LOG_HEARTBEAT_S = 1.0

PITCH_LINE = -20
PITCH_SCAN = 5
SCAN_YAW_SPEED = 180.0
SCAN_TURNS = 1
SCAN_TURNS_AFTER_LOST = 1

LINE_SPEED = 0.35
LINE_PID_KP = 80.0
LINE_PID_OUT_MAX = 80.0

# --- 瞄准 PID（参考 DJI 线/目标跟踪：较大 P、小 D、I=0；输出为云台角速度）---
# 示例循线常用 set_ctrl_params(330,0,28)，误差约在 ±0.5；
# 行人跟踪略降增益，减少到中心后的抖动。
AIM_YAW_KP = 90.0
AIM_YAW_KI = 0.0
AIM_YAW_KD = 25.0
AIM_YAW_OUT_MAX = 80.0
AIM_PITCH_KP = 70.0
AIM_PITCH_KI = 0.0
AIM_PITCH_KD = 20.0
AIM_PITCH_OUT_MAX = 50.0
AIM_DEADZONE = 0.06       # |err| 小于此则角速度置 0（DJI 式死区，防抖）
AIM_OK_ERR = 0.08
PERSON_MIN_W = 0.06       # 归一化框宽高下限，过滤噪点
PERSON_MIN_H = 0.08

T_AIM_BEFORE_IR = 1.0
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

g_state = STATE_INIT
g_state_t0 = 0.0
g_no_person_t0 = 0.0
g_person_miss = 0
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
    # 死区：误差很小时直接 0，避免在中心附近来回抖
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
# VISION — 行人（严格）/ 线
# =============================================================================
def people_get_first():
    """
    返回 (ok, x, y, w, h)。
    兼容 list 0 基与部分 RmList 1 基；要求框足够大才算有效目标。
    """
    info = vision_ctrl.get_people_detection_info()
    n = None
    x = 0.5
    y = 0.5
    w = 0.0
    h = 0.0
    # 0-based: N,X,Y,W,H
    try:
        n = info[0]
        x = info[1]
        y = info[2]
        w = info[3]
        h = info[4]
    except Exception:
        # 1-based 尝试
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
    # 有的固件 N 是 float
    try:
        ni = int(n)
    except Exception:
        return False, 0.5, 0.5, 0.0, 0.0
    if ni < 1:
        return False, 0.5, 0.5, 0.0, 0.0
    # 过滤过小噪声框
    try:
        if w < PERSON_MIN_W or h < PERSON_MIN_H:
            return False, x, y, w, h
    except Exception:
        return False, 0.5, 0.5, 0.0, 0.0
    # 中心应在画面合理范围
    if x < 0.0 or x > 1.0 or y < 0.0 or y > 1.0:
        return False, x, y, w, h
    return True, x, y, w, h

def people_seen():
    ok, x, y, w, h = people_get_first()
    return ok

def person_track_update():
    """
    更新丢失计数。返回 True=当前帧有效看到人。
    丢失时立刻停云台，避免继续 PID 抖。
    """
    global g_person_miss, g_no_person_t0
    ok, x, y, w, h = people_get_first()
    if ok:
        g_person_miss = 0
        g_no_person_t0 = now_s()
        return True
    # 当前帧无人
    g_person_miss = g_person_miss + 1
    gimbal_stop()
    return False

def person_confirmed_lost():
    """连续 miss 足够帧，且已持续至少 T_CLEAR 秒。"""
    if g_person_miss < PERSON_MISS_NEED:
        return False
    # g_no_person_t0 在最后一次见到人时刷新；未见人时不刷新
    # 若从未见过：set_state 时已设 g_no_person_t0
    if (now_s() - g_no_person_t0) < T_CLEAR:
        return False
    return True

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
        extra = " deg=%.0f/%.0f pct=%.0f person=%s miss=%d" % (
            g_scan_deg_done, g_scan_need_deg, pct, str(has_p), g_person_miss
        )
    elif g_state == STATE_LOCK or g_state == STATE_FIRE:
        extra = " person=%s miss=%d xy=(%.2f,%.2f) phase=%d" % (
            str(has_p), g_person_miss, px, py, g_fire_phase
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
    gimbal_ctrl.rotate_with_speed(0, 0)
    gimbal_ctrl.stop()

def gimbal_scan_reset_pose():
    gimbal_stop()
    gimbal_ctrl.recenter()
    time.sleep(0.15)
    gimbal_set_pitch_scan()

def chassis_halt():
    chassis_ctrl.stop()

def aim_pid_towards_person(dt):
    """
    仅在有效检测到人时驱动云台；否则立即停转并返回 has=False。
    PID 参考 DJI：P+D、死区、输出限幅。
    """
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
    # 已对准：明确停转，不要带着噪声输出
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
    global g_fire_count
    if ENABLE_FIRE == False:
        log("IR_WARN skip ENABLE_FIRE=0")
        return
    gun_ctrl.set_fire_count(1)
    gun_ctrl.fire_once()
    g_fire_count = g_fire_count + 1
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
# STATE MACHINE
# =============================================================================
def set_state(s, reason):
    global g_state, g_state_t0, g_no_person_t0, g_person_miss
    global g_patrol_line_t0, g_scan_deg_done, g_scan_need_deg
    global g_fire_count, g_fire_phase, g_phase_t0, g_ir_done
    old = g_state
    g_state = s
    g_state_t0 = now_s()
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
        gimbal_stop()
        pid_reset_aim()
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
        g_person_miss = 0
        g_no_person_t0 = now_s()
        ok, x, y, w, h = people_get_first()
        log("LOCK ok=%s xy=(%.2f,%.2f) wh=(%.2f,%.2f)" % (str(ok), x, y, w, h))

    if s == STATE_FIRE:
        chassis_halt()
        pid_reset_aim()
        leds_alert_red()
        g_fire_phase = FIRE_PHASE_AIM
        g_phase_t0 = now_s()
        g_ir_done = False
        g_person_miss = 0
        g_no_person_t0 = now_s()
        log("FIRE enter")

    if s == STATE_RECOVER:
        fire_stop()
        chassis_halt()
        gimbal_stop()
        pid_reset_aim()
        gimbal_set_pitch_line()
        leds_normal()
        log("RECOVER find line")

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
        log("PATROL line ok follow %.1fs" % T_MOVE)
    line_follow_step()
    if (now_s() - g_patrol_line_t0) >= T_MOVE:
        g_patrol_line_t0 = 0.0
        set_state(STATE_SCAN, "follow_time_up")

def tick_scan_common(after_lost):
    global g_scan_deg_done, g_scan_need_deg
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
    gimbal_stop()
    if after_lost:
        log("LOST_SCAN done -> RECOVER")
        set_state(STATE_RECOVER, "lost_scan_done")
        return
    if line_seen():
        log("SCAN done has line -> PATROL")
        set_state(STATE_PATROL, "scan_has_line")
        return
    log("SCAN done no line -> recenter rescan")
    gimbal_scan_reset_pose()
    g_scan_deg_done = 0.0
    g_scan_need_deg = 360.0 * SCAN_TURNS
    log("SCAN restart need=%.0f" % g_scan_need_deg)

def tick_scan():
    tick_scan_common(False)

def tick_lost_scan():
    tick_scan_common(True)

def tick_lock():
    """
    锁定：有效检测才 PID；丢失确认后离开，禁止无人仍瞄准。
    """
    global g_ir_done, g_fire_phase, g_phase_t0
    chassis_halt()
    has = person_track_update()
    if has:
        aligned, _ = aim_pid_towards_person(LOOP_DT)
        if g_ir_done == False and state_age() >= T_AIM_BEFORE_IR:
            fire_ir_warn_once()
            g_ir_done = True
            g_fire_phase = FIRE_PHASE_IR_DONE
            g_phase_t0 = now_s()
            log("LOCK IR warn -> FIRE")
            set_state(STATE_FIRE, "after_ir_warn")
        return
    # 无人：已在 person_track_update 里 stop 云台
    if person_confirmed_lost():
        log("LOCK lost miss=%d -> LOST_SCAN" % g_person_miss)
        set_state(STATE_LOST_SCAN, "person_lost")

def tick_fire():
    global g_ir_done, g_fire_phase, g_phase_t0
    chassis_halt()
    has = person_track_update()
    if has == False:
        fire_stop()
        if person_confirmed_lost():
            log("FIRE lost miss=%d -> LOST_SCAN" % g_person_miss)
            set_state(STATE_LOST_SCAN, "person_lost_fire")
        return

    # 仍有人：继续 PID（有死区）
    aim_pid_towards_person(LOOP_DT)

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
            g_fire_phase = FIRE_PHASE_BURST_ON
            g_phase_t0 = now_s()
            fire_bead_burst_start()
        return

def tick_recover():
    fire_stop()
    chassis_halt()
    gimbal_stop()
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
    pid_reset_aim()
    log("setup done v1.3.0 pid soft + person miss debounce")

def start():
    global g_state
    print("======== Line Guard start ========")
    print("# LINE_GUARD_VERSION=1.3.0 stamp=2026-08-01 21:25:32")
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
