# LINE_GUARD_VERSION=1.9.0 stamp=2026-08-03 17:05:30  (paste this whole file; check stamp matches latest)
# -*- coding: utf-8 -*-
# S1 Line Guard v1.9 — 单文件粘贴进 App 实验室
#
# =============================================================================
# 总览：状态机
# =============================================================================
#
#   INIT ──boot──► PATROL
#                    │
#        ┌───────────┼───────────────┐
#        │ 有人(3帧) │ 丢线/到时      │
#        ▼          ▼               │
#      LOCK ◄──── SCAN              │
#        │          │ 扫完有线        │
#        │ 跟瞄+IR  │────────► PATROL│
#        ▼          │ 扫完无线        │
#      FIRE         └───────► SCAN（完整两遍）
#        │  连发2s → 等待3s → 连发2s …（人仍在）
#        │ 确认丢人
#        ▼
#    LOST_SCAN ── 快回中 → 完整两遍 SCAN
#        │
#        ▼
#    RECOVER ──有线──► PATROL
#        └─仍无线──► SCAN
#
# 打断：
#   - SCAN 中途「连续 3 帧」见人 → LOCK（先停转跟瞄，不回中）
#   - LOCK/FIRE 确认丢人 → LOST_SCAN：快回中 + 完整两遍（不续半段）
#
# =============================================================================
# SCAN 轨迹（yaw 硬件约 ±250°，单边 180°）
# =============================================================================
#   队列 [ +180, 0, -180, 0 ]，扫描角速度 150°/s
#   若顺/逆与实车相反：对调 SCAN_SIDE_A / SCAN_SIDE_B
#
# 检测抗抖：
#   - 进 LOCK：连续 PERSON_HIT_NEED=3 帧
#   - 跟丢：连续 miss + T_CLEAR；丢检帧不立刻停云台，用上一帧位置 coast 跟瞄
#
# 射击：
#   - LOCK 发现音响一次（recognize_success）
#   - 进 FIRE 前红外 ir_blaster 示警 1 次（仅灯效，不配射击音）
#   - 水弹 fire_continuous 连发 2s → stop → 人仍在则等 3s → 再连发 2s
#   - 水弹段不播放 media_sound_shoot（机身自带射击声）
#
# =============================================================================

# =============================================================================
# CONFIG
# =============================================================================
T_MOVE = 3.0
T_CLEAR = 1.5
PERSON_MISS_NEED = 18
PERSON_HIT_NEED = 3
LOOP_DT = 0.05
LOG_HEARTBEAT_S = 1.0

PITCH_LINE = -20
PITCH_SCAN = 10

# --- SCAN ---
SCAN_HALF = 180.0
SCAN_SIDE_A = SCAN_HALF
SCAN_SIDE_B = -SCAN_HALF
SCAN_YAW_SPEED = 150.0
YAW_ARRIVE = 8.0
SCAN_STUCK_FRAMES = 12
HOME_YAW_SPEED = 500.0
HOME_TIMEOUT_S = 3.0

LINE_SPEED = 0.35
LINE_PID_KP = 80.0
LINE_PID_OUT_MAX = 80.0
LINE_CONFIRM_FRAMES = 8

AIM_YAW_KP = 70.0
AIM_YAW_KI = 0.0
AIM_YAW_KD = 22.0
AIM_YAW_OUT_MAX = 50.0
AIM_PITCH_KP = 50.0
AIM_PITCH_KI = 0.0
AIM_PITCH_KD = 16.0
AIM_PITCH_OUT_MAX = 30.0
AIM_DEADZONE = 0.08
AIM_OK_ERR = 0.10
PERSON_MIN_W = 0.06
PERSON_MIN_H = 0.08

# LOCK 跟瞄多久后红外示警 → FIRE
T_AIM_BEFORE_IR = 1.2
# 水弹：连发 2s，间隔 3s（人未离开才进入下一轮）
T_BURST_ON = 2.0
T_BURST_WAIT = 3.0

ENABLE_FIRE = True
FORCE_NO_LINE = False
FLASH_HZ = 3

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
FIRE_PHASE_AIM = 0       # 未完成 IR（兜底）
FIRE_PHASE_IR_DONE = 1   # IR 已完成，即将/正在连发
FIRE_PHASE_BURST_ON = 2  # 水弹连发中
FIRE_PHASE_BURST_WAIT = 3  # 连发间隔等待

g_state = STATE_INIT
g_state_t0 = 0.0
g_no_person_t0 = 0.0
g_person_miss = 0
g_person_hit = 0
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

# 上一帧有效人体框（丢检时 coast 跟瞄用）
g_last_px = 0.5
g_last_py = 0.5
g_have_last_person = False

# SCAN
g_scan_queue = []
g_scan_qi = 0
g_scan_target_yaw = 0.0
g_scan_last_yaw = 0.0
g_scan_stuck = 0
g_scan_seg_name = ""

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

def get_yaw():
    return gimbal_ctrl.get_axis_angle(rm_define.gimbal_axis_yaw)

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

# =============================================================================
# 灯光 + 内置音效
#
#  | 状态       | 灯                    | 音效                          |
#  | PATROL     | 绿常亮                | solmization_2C                |
#  | SCAN       | 蓝快闪+顶跑马         | scanning                      |
#  | RECENTER   | 黄呼吸                | gimbal_rotate                 |
#  | LOCK       | 紫快闪                | recognize_success（进态1次）  |
#  | FIRE IR    | 橙快闪                | 无（不配 countdown）          |
#  | FIRE 连发  | 红快闪+枪口灯         | 无（水弹自带声）              |
#  | FIRE 等待  | 橙常亮                | 无                            |
#  | LOST       | 橙呼吸                | attacked                      |
#  | RECOVER    | 白慢闪                | solmization_1G                |
# =============================================================================
def sfx(sound_enum):
    try:
        media_ctrl.play_sound(sound_enum)
    except Exception:
        pass

def leds_off():
    try:
        led_ctrl.gun_led_off()
    except Exception:
        pass
    try:
        led_ctrl.turn_off(rm_define.armor_all)
    except Exception:
        pass

def leds_set(r, g, b, effect, flash_hz, top_marquee=False, gun_on=False):
    leds_off()
    if gun_on:
        try:
            led_ctrl.gun_led_on()
        except Exception:
            pass
    led_ctrl.set_bottom_led(rm_define.armor_bottom_all, r, g, b, effect)
    if top_marquee:
        try:
            led_ctrl.set_top_led(rm_define.armor_top_all, r, g, b, rm_define.effect_marquee)
        except Exception:
            led_ctrl.set_top_led(rm_define.armor_top_all, r, g, b, effect)
    else:
        led_ctrl.set_top_led(rm_define.armor_top_all, r, g, b, effect)
    if effect == rm_define.effect_flash:
        try:
            led_ctrl.set_flash(rm_define.armor_all, flash_hz)
        except Exception:
            pass

def fx_patrol():
    leds_set(0, 255, 0, rm_define.effect_always_on, FLASH_HZ)
    sfx(rm_define.media_sound_solmization_2C)

def fx_scan():
    leds_set(0, 80, 255, rm_define.effect_flash, 5, top_marquee=True)
    sfx(rm_define.media_sound_scanning)

def fx_scan_seg(seg_name):
    leds_set(0, 80, 255, rm_define.effect_flash, 5, top_marquee=True)
    sfx(rm_define.media_sound_scanning)
    log("SCAN seg start: %s" % seg_name)

def fx_recenter():
    leds_set(255, 200, 0, rm_define.effect_breath, FLASH_HZ)
    sfx(rm_define.media_sound_gimbal_rotate)

def fx_lock():
    """发现人：紫闪 + 识别成功音（仅进 LOCK 时调用一次）。"""
    leds_set(200, 0, 255, rm_define.effect_flash, 6)
    sfx(rm_define.media_sound_recognize_success)

def fx_fire_ir_led():
    """红外示警：仅橙闪，不播音效。"""
    leds_set(255, 100, 0, rm_define.effect_flash, 7)

def fx_fire_burst_led():
    """水弹连发：红闪+枪口灯，不播 shoot 音（机身自带）。"""
    leds_set(255, 0, 0, rm_define.effect_flash, 8, gun_on=True)

def fx_fire_wait_led():
    """连发间隔等待：橙常亮，静音。"""
    leds_set(255, 140, 0, rm_define.effect_always_on, FLASH_HZ)

def fx_person_lost():
    leds_set(255, 140, 0, rm_define.effect_breath, FLASH_HZ)
    sfx(rm_define.media_sound_attacked)

def fx_recover():
    leds_set(255, 255, 255, rm_define.effect_flash, 2)
    sfx(rm_define.media_sound_solmization_1G)

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

def person_hit_reset():
    global g_person_hit
    g_person_hit = 0

def person_hit_update():
    """
    进 LOCK 前的防抖：连续 PERSON_HIT_NEED 帧检出才算确认。
    返回 True = 已确认见人，应进入 LOCK。
    """
    global g_person_hit
    if people_seen():
        g_person_hit = g_person_hit + 1
    else:
        g_person_hit = 0
    return g_person_hit >= PERSON_HIT_NEED

def person_track_update():
    """
    LOCK/FIRE 跟瞄用。
    - 检出：更新 last_xy，清 miss
    - 丢检：miss++，**不立刻停云台**（避免闪一下就停导致跟丢）
    返回 True=本帧有检出。
    """
    global g_person_miss, g_no_person_t0
    global g_last_px, g_last_py, g_have_last_person
    ok, x, y, w, h = people_get_first()
    if ok:
        g_person_miss = 0
        g_no_person_t0 = now_s()
        g_last_px = x
        g_last_py = y
        g_have_last_person = True
        return True
    g_person_miss = g_person_miss + 1
    return False

def person_confirmed_lost():
    if g_person_miss < PERSON_MISS_NEED:
        return False
    if (now_s() - g_no_person_t0) < T_CLEAR:
        return False
    return True

def aim_pid_towards_xy(x, y, dt):
    """向归一化图像坐标 (x,y) 做 PID 云台。"""
    global g_iy, g_ey_prev, g_ip, g_ep_prev
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
        return True
    gimbal_ctrl.rotate_with_speed(yaw_spd, -pitch_spd)
    return False

def aim_pid_track(dt):
    """
    有检出用当前框；无检出但有 last 则 coast 跟 last（扛闪烁）。
    确认丢失前不强制停台。
    """
    ok, x, y, w, h = people_get_first()
    if ok:
        return aim_pid_towards_xy(x, y, dt), True
    if g_have_last_person:
        return aim_pid_towards_xy(g_last_px, g_last_py, dt), False
    gimbal_stop()
    return False, False

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
        extra = " lineHit=%d miss=%d pHit=%d follow=%.1f" % (
            g_line_hit, g_line_miss, g_person_hit, age
        )
    elif g_state == STATE_SCAN or g_state == STATE_LOST_SCAN:
        extra = " qi=%d/%d seg=%s tgt=%.0f yaw=%.0f pHit=%d person=%s" % (
            g_scan_qi, len(g_scan_queue), g_scan_seg_name, g_scan_target_yaw,
            get_yaw(), g_person_hit, str(has_p)
        )
    elif g_state == STATE_LOCK or g_state == STATE_FIRE:
        extra = " person=%s miss=%d xy=(%.2f,%.2f) fphase=%d ir=%s" % (
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

def set_gimbal_speed(spd):
    try:
        gimbal_ctrl.set_rotate_speed(spd)
    except Exception:
        pass

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
    try:
        ir_blaster_ctrl.stop()
    except Exception:
        pass
    try:
        led_ctrl.gun_led_off()
    except Exception:
        pass

def fire_ir_warn_once():
    """红外示警一次：橙灯，无射击配音。"""
    global g_fire_count, g_ir_done
    if g_ir_done:
        log("IR_WARN skip already")
        return
    fx_fire_ir_led()
    if ENABLE_FIRE == False:
        log("IR_WARN skip ENABLE_FIRE=0")
        g_ir_done = True
        return
    ok_ir = False
    try:
        ir_blaster_ctrl.set_fire_count(1)
        ir_blaster_ctrl.fire_once()
        ok_ir = True
        log("IR_WARN ir_blaster fire_once")
    except Exception:
        ok_ir = False
    if ok_ir == False:
        # 无 ir_blaster 时退回水弹单发一次作示警（仍不播配音）
        gun_ctrl.set_fire_count(1)
        gun_ctrl.fire_once()
        log("IR_WARN fallback gun fire_once")
    g_fire_count = g_fire_count + 1
    g_ir_done = True

def fire_bead_burst_start():
    """水弹连发开始：仅灯效，不播 media_sound_shoot。"""
    if ENABLE_FIRE == False:
        log("BURST_ON skip ENABLE_FIRE=0")
        return
    fx_fire_burst_led()
    gun_ctrl.set_fire_count(1)
    gun_ctrl.fire_continuous()
    log("BURST_ON %.1fs" % T_BURST_ON)

def fire_bead_burst_stop():
    gun_ctrl.stop()
    try:
        led_ctrl.gun_led_off()
    except Exception:
        pass
    log("BURST_STOP -> wait %.1fs" % T_BURST_WAIT)

# =============================================================================
# SCAN 核心：完整两遍 [ +180, 0, -180, 0 ]
# =============================================================================
def scan_queue_full_two_rounds():
    return [SCAN_SIDE_A, 0.0, SCAN_SIDE_B, 0.0]

def scan_seg_label(qi, target):
    if qi == 0:
        return "R1_to_%+.0f" % target
    if qi == 1:
        return "R1_home_0"
    if qi == 2:
        return "R2_to_%+.0f" % target
    if qi == 3:
        return "R2_home_0"
    return "qi%d_to_%+.0f" % (qi, target)

def gimbal_fast_home(reason, keep_scan_pitch=False):
    gimbal_stop()
    fx_recenter()
    yaw0 = get_yaw()
    log("HOME begin yaw=%.0f | %s" % (yaw0, reason))
    set_gimbal_speed(HOME_YAW_SPEED)
    try:
        gimbal_ctrl.yaw_ctrl(0)
    except Exception:
        t0 = now_s()
        while (now_s() - t0) < HOME_TIMEOUT_S:
            y = get_yaw()
            if abs(y) <= YAW_ARRIVE:
                break
            if y > 0:
                gimbal_ctrl.rotate_with_speed(-min(SCAN_YAW_SPEED, 250), 0)
            else:
                gimbal_ctrl.rotate_with_speed(min(SCAN_YAW_SPEED, 250), 0)
            time.sleep(LOOP_DT)
        gimbal_stop()
    if keep_scan_pitch == False:
        try:
            gimbal_ctrl.pitch_ctrl(PITCH_LINE)
        except Exception:
            pass
    time.sleep(0.08)
    if abs(get_yaw()) > 12.0:
        set_gimbal_speed(HOME_YAW_SPEED)
        try:
            gimbal_ctrl.yaw_ctrl(0)
        except Exception:
            pass
    gimbal_stop()
    log("HOME done yaw=%.0f" % get_yaw())

def scan_load_segment(qi):
    global g_scan_qi, g_scan_target_yaw, g_scan_last_yaw, g_scan_stuck, g_scan_seg_name
    g_scan_qi = qi
    g_scan_target_yaw = g_scan_queue[qi]
    g_scan_last_yaw = get_yaw()
    g_scan_stuck = 0
    g_scan_seg_name = scan_seg_label(qi, g_scan_target_yaw)
    if abs(g_scan_target_yaw) < 0.1:
        fx_recenter()
        log("SCAN seg qi=%d %s (回中段)" % (qi, g_scan_seg_name))
    else:
        fx_scan_seg(g_scan_seg_name)
        log("SCAN seg qi=%d %s yaw0=%.0f" % (qi, g_scan_seg_name, g_scan_last_yaw))

def scan_start_full(reason):
    global g_scan_queue
    g_scan_queue = scan_queue_full_two_rounds()
    person_hit_reset()
    gimbal_set_pitch_scan()
    time.sleep(0.05)
    if abs(get_yaw()) > YAW_ARRIVE:
        gimbal_fast_home("scan_start_ensure_center", keep_scan_pitch=True)
        gimbal_set_pitch_scan()
        time.sleep(0.05)
    if len(g_scan_queue) <= 0:
        log("SCAN empty queue")
        return
    fx_scan()
    scan_load_segment(0)
    log("SCAN full 2-round start n=%d A=%+.0f B=%+.0f spd=%.0f | %s" % (
        len(g_scan_queue), SCAN_SIDE_A, SCAN_SIDE_B, SCAN_YAW_SPEED, reason
    ))

def scan_tick_turn():
    global g_scan_last_yaw, g_scan_stuck
    yaw = get_yaw()
    err = g_scan_target_yaw - yaw
    if abs(err) <= YAW_ARRIVE:
        log("SCAN arrive yaw=%.0f tgt=%.0f seg=%s" % (yaw, g_scan_target_yaw, g_scan_seg_name))
        return True
    d = yaw - g_scan_last_yaw
    if abs(d) < 0.35:
        g_scan_stuck = g_scan_stuck + 1
    else:
        g_scan_stuck = 0
    g_scan_last_yaw = yaw
    if g_scan_stuck >= SCAN_STUCK_FRAMES:
        log("SCAN stuck yaw=%.0f tgt=%.0f -> done" % (yaw, g_scan_target_yaw))
        return True
    spd = SCAN_YAW_SPEED
    if spd < 1.0:
        spd = 1.0
    if err > 0:
        gimbal_ctrl.rotate_with_speed(spd, 0)
    else:
        gimbal_ctrl.rotate_with_speed(-spd, 0)
    return False

def scan_advance_or_finish():
    global g_scan_qi
    gimbal_stop()
    ni = g_scan_qi + 1
    if ni < len(g_scan_queue):
        scan_load_segment(ni)
        return "next"
    return "done"

# =============================================================================
# STATE MACHINE
# =============================================================================
def set_state(s, reason):
    global g_state, g_state_t0, g_no_person_t0, g_person_miss
    global g_patrol_line_t0, g_fire_count, g_fire_phase, g_phase_t0
    global g_ir_done, g_line_hit, g_line_miss, g_have_last_person
    global g_last_px, g_last_py
    old = g_state
    g_state = s
    g_state_t0 = now_s()
    if s != STATE_FIRE:
        g_no_person_t0 = now_s()
        g_person_miss = 0
    log("STATE %s -> %s | %s" % (state_name(old), state_name(s), reason))

    # ----- PATROL -----
    if s == STATE_PATROL:
        fire_stop()
        chassis_halt()
        gimbal_stop()
        robot_ctrl.set_mode(rm_define.robot_mode_free)
        gimbal_set_pitch_line()
        fx_patrol()
        g_patrol_line_t0 = 0.0
        g_line_hit = 0
        g_line_miss = 0
        person_hit_reset()
        vision_ctrl.enable_detection(rm_define.vision_detection_line)

    # ----- SCAN：完整两遍 -----
    if s == STATE_SCAN:
        fire_stop()
        chassis_halt()
        robot_ctrl.set_mode(rm_define.robot_mode_free)
        scan_start_full("normal_scan")

    # ----- LOST_SCAN：快回中 + 完整两遍 -----
    if s == STATE_LOST_SCAN:
        fire_stop()
        chassis_halt()
        gimbal_stop()
        pid_reset_aim()
        g_have_last_person = False
        robot_ctrl.set_mode(rm_define.robot_mode_free)
        fx_person_lost()
        time.sleep(0.2)
        gimbal_fast_home("lost_before_rescan", keep_scan_pitch=True)
        scan_start_full("lost_rescan_full")

    # ----- LOCK：发现音一次，停转跟瞄 -----
    if s == STATE_LOCK:
        fire_stop()
        chassis_halt()
        gimbal_stop()
        robot_ctrl.set_mode(rm_define.robot_mode_free)
        gimbal_set_pitch_scan()
        pid_reset_aim()
        person_hit_reset()
        fx_lock()
        g_fire_count = 0
        g_ir_done = False
        g_fire_phase = FIRE_PHASE_AIM
        g_phase_t0 = now_s()
        g_person_miss = 0
        g_no_person_t0 = now_s()
        ok, x, y, w, h = people_get_first()
        if ok:
            g_last_px = x
            g_last_py = y
            g_have_last_person = True
        log("LOCK ok=%s xy=(%.2f,%.2f) hitNeed=%d" % (str(ok), x, y, PERSON_HIT_NEED))

    # ----- FIRE：IR 已做则直接进入连发节奏 -----
    if s == STATE_FIRE:
        chassis_halt()
        pid_reset_aim()
        if g_ir_done:
            g_fire_phase = FIRE_PHASE_IR_DONE
        else:
            g_fire_phase = FIRE_PHASE_AIM
            fx_lock()
        g_phase_t0 = now_s()
        g_person_miss = 0
        g_no_person_t0 = now_s()
        log("FIRE enter ir_done=%s burst=%.1fs wait=%.1fs" % (
            str(g_ir_done), T_BURST_ON, T_BURST_WAIT
        ))

    # ----- RECOVER -----
    if s == STATE_RECOVER:
        fire_stop()
        chassis_halt()
        gimbal_stop()
        pid_reset_aim()
        gimbal_set_pitch_line()
        fx_recover()
        g_line_hit = 0
        g_line_miss = 0
        person_hit_reset()
        log("RECOVER find line")

def tick_patrol():
    """
    PATROL:
      连续 3 帧见人 → LOCK
      稳定无蓝线 → SCAN
      稳定有蓝线循线 T_MOVE → SCAN
    """
    global g_patrol_line_t0
    line_update()
    if person_hit_update():
        log("PATROL person hit=%d -> LOCK" % g_person_hit)
        set_state(STATE_LOCK, "person_on_patrol")
        return
    if line_stable_false():
        g_patrol_line_t0 = 0.0
        person_hit_reset()
        chassis_halt()
        set_state(STATE_SCAN, "no_line_stable")
        return
    if line_stable_true() == False:
        chassis_halt()
        return
    if g_patrol_line_t0 <= 0.0:
        g_patrol_line_t0 = now_s()
        log("PATROL line stable follow %.1fs" % T_MOVE)
    line_follow_step()
    if (now_s() - g_patrol_line_t0) >= T_MOVE:
        g_patrol_line_t0 = 0.0
        person_hit_reset()
        set_state(STATE_SCAN, "follow_time_up")

def tick_scan_common(is_lost):
    """
    SCAN / LOST_SCAN:
      连续 3 帧见人 → 停转 → LOCK
      段推进 / 整表完成同 v1.8
    """
    chassis_halt()
    if person_hit_update():
        gimbal_stop()
        log("SCAN person hit=%d -> LOCK" % g_person_hit)
        set_state(STATE_LOCK, "person_on_scan")
        return
    if scan_tick_turn() == False:
        return
    # 段切换时清命中计数，避免跨段误累加
    person_hit_reset()
    log("SCAN seg done %s yaw=%.0f" % (g_scan_seg_name, get_yaw()))
    adv = scan_advance_or_finish()
    if adv == "next":
        return
    gimbal_fast_home("scan_two_rounds_done", keep_scan_pitch=False)
    if is_lost:
        log("LOST_SCAN done 2-round -> RECOVER")
        set_state(STATE_RECOVER, "lost_scan_done")
        return
    i = 0
    while i < LINE_CONFIRM_FRAMES:
        line_update()
        time.sleep(LOOP_DT)
        i = i + 1
    if line_stable_true():
        log("SCAN done line -> PATROL")
        set_state(STATE_PATROL, "scan_has_line")
        return
    log("SCAN done no line -> SCAN again")
    set_state(STATE_SCAN, "rescan_no_line")

def tick_scan():
    tick_scan_common(False)

def tick_lost_scan():
    tick_scan_common(True)

def tick_lock():
    """
    LOCK:
      跟瞄（丢检 coast）；够 T_AIM_BEFORE_IR → IR 示警 → FIRE
      确认丢人 → LOST_SCAN
    """
    global g_fire_phase, g_phase_t0
    chassis_halt()
    person_track_update()
    aim_pid_track(LOOP_DT)
    if person_confirmed_lost():
        log("LOCK lost miss=%d -> LOST_SCAN" % g_person_miss)
        set_state(STATE_LOST_SCAN, "person_lost")
        return
    if g_ir_done == False and state_age() >= T_AIM_BEFORE_IR:
        # 至少近期见过人再开火示警（have last 或 miss 不多）
        if g_have_last_person and g_person_miss < PERSON_MISS_NEED:
            fire_ir_warn_once()
            g_fire_phase = FIRE_PHASE_IR_DONE
            g_phase_t0 = now_s()
            log("LOCK IR warn -> FIRE")
            set_state(STATE_FIRE, "after_ir_warn")

def tick_fire():
    """
    FIRE:
      有人/coast：PID 跟瞄
      确认丢人 → 停枪 → LOST_SCAN
      节奏：连发 T_BURST_ON(2s) → 等待 T_BURST_WAIT(3s) → 再连发 …
      水弹段不播配音
    """
    global g_fire_phase, g_phase_t0
    chassis_halt()
    person_track_update()
    if person_confirmed_lost():
        fire_stop()
        log("FIRE lost miss=%d -> LOST_SCAN" % g_person_miss)
        set_state(STATE_LOST_SCAN, "person_lost_fire")
        return
    aim_pid_track(LOOP_DT)

    # 兜底：若未 IR（异常路径）
    if g_ir_done == False:
        if state_age() >= T_AIM_BEFORE_IR:
            fire_ir_warn_once()
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
            fx_fire_wait_led()
            g_fire_phase = FIRE_PHASE_BURST_WAIT
            g_phase_t0 = now_s()
            log("FIRE wait %.1fs (person still tracked)" % T_BURST_WAIT)
        return

    if g_fire_phase == FIRE_PHASE_BURST_WAIT:
        # 等待期间人走了 → 上面 confirmed_lost 已处理
        if phase_age() >= T_BURST_WAIT:
            g_fire_phase = FIRE_PHASE_BURST_ON
            g_phase_t0 = now_s()
            fire_bead_burst_start()
        return

def tick_recover():
    fire_stop()
    chassis_halt()
    gimbal_stop()
    gimbal_set_pitch_line()
    line_update()
    if line_stable_true():
        log("RECOVER line -> PATROL")
        set_state(STATE_PATROL, "line_found")
        return
    if line_stable_false() and state_age() >= 0.6:
        log("RECOVER no line -> SCAN")
        set_state(STATE_SCAN, "still_no_line")
        return

# =============================================================================
# ENTRY
# =============================================================================
def setup():
    log("setup begin")
    robot_ctrl.set_mode(rm_define.robot_mode_free)
    chassis_halt()
    set_gimbal_speed(HOME_YAW_SPEED)
    gimbal_ctrl.yaw_ctrl(0)
    gimbal_ctrl.pitch_ctrl(PITCH_LINE)
    time.sleep(0.25)
    vision_ctrl.enable_detection(rm_define.vision_detection_people)
    vision_ctrl.enable_detection(rm_define.vision_detection_line)
    vision_ctrl.line_follow_color_set(rm_define.line_follow_color_blue)
    media_ctrl.exposure_value_update(rm_define.exposure_value_medium)
    gun_ctrl.set_fire_count(1)
    try:
        ir_blaster_ctrl.set_fire_count(1)
    except Exception:
        pass
    fx_patrol()
    pid_reset_aim()
    person_hit_reset()
    log("setup done v1.9.0 scan=%.0f hit=%d burst=%.1f wait=%.1f" % (
        SCAN_YAW_SPEED, PERSON_HIT_NEED, T_BURST_ON, T_BURST_WAIT
    ))

def start():
    global g_state
    print("======== Line Guard start ========")
    print("# LINE_GUARD_VERSION=1.9.0 stamp=2026-08-03 17:05:30")
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
