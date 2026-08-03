# LINE_GUARD_VERSION=1.7.0 stamp=2026-08-03 15:38:49  (paste this whole file; check stamp matches latest)
# -*- coding: utf-8 -*-
# S1 Line Guard v1.7 — 单文件粘贴进 App 实验室
#
# SCAN（云台 yaw 硬件约 ±250，无法单方向连转真 360°）：
#   1) 快速 PREP 到左限 -LIM（不算一圈，避免「从 0 只转 ~230° 就换向」）
#   2) PASS1：-LIM → +LIM（满行程 ~490°，覆盖整片 FOV）
#   3) PASS2：+LIM → -LIM
#   4) 高速 yaw_ctrl(0) 回中 + 低头巡线俯仰
# 打断：见人 LOCK；离开后按断点续扫
# 状态：上下装甲灯颜色/闪法 + 内置音效 明显区分（见 fx_*）
# 射击：红外 ir_blaster 示警一次；水弹 gun 连发1s/停1s

# =============================================================================
# CONFIG
# =============================================================================
T_MOVE = 3.0
T_CLEAR = 1.0
PERSON_MISS_NEED = 10
LOOP_DT = 0.05
LOG_HEARTBEAT_S = 1.0

PITCH_LINE = -20
PITCH_SCAN = 10
SCAN_YAW_SPEED = 200.0
# 软限位：尽量扫满可用行程（硬件约 ±250）
YAW_LIM = 245.0
YAW_ARRIVE = 8.0
SCAN_STUCK_FRAMES = 12
# 回中角速度（°/s）；默认 ~30 会从极限回中很慢
HOME_YAW_SPEED = 500.0
PREP_YAW_SPEED = 400.0
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

T_AIM_BEFORE_IR = 1.2
T_BURST_ON = 1.0
T_BURST_OFF = 1.0

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

FIRE_PHASE_AIM = 0
FIRE_PHASE_IR_DONE = 1
FIRE_PHASE_BURST_ON = 2
FIRE_PHASE_BURST_OFF = 3

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

# SCAN: 队列为绝对目标 yaw（度）
g_scan_queue = []
g_scan_qi = 0
g_scan_target_yaw = 0.0
g_scan_last_yaw = 0.0
g_scan_stuck = 0
g_brk_valid = False
g_brk_target_yaw = 0.0
g_scan_pass = 0

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
# 灯光 + 内置音效（状态标识，颜色/闪法/声音都不同）
#
#  | 状态      | 灯色/效果              | 内置音效                |
#  | PATROL    | 绿 常亮                | solmization_2C          |
#  | SCAN      | 纯蓝 快闪 + 顶灯跑马   | scanning                |
#  | RECENTER  | 黄 呼吸                | gimbal_rotate           |
#  | LOCK      | 紫 快闪                | recognize_success       |
#  | FIRE 示警 | 橙 快闪                | count_down              |
#  | FIRE 射击 | 红 极快闪 + 枪口灯     | shoot                   |
#  | LOST      | 橙 呼吸                | attacked                |
#  | RECOVER   | 白 慢闪                | solmization_1G          |
# =============================================================================
def sfx(sound_enum):
    """播放内置音效，不阻塞控制环。"""
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
    """统一设上下装甲灯；切换状态时先关再开，避免闪烁模式残留。"""
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
    # 巡线：纯绿常亮 = 安全巡逻（与 SCAN 蓝闪强对比）
    leds_set(0, 255, 0, rm_define.effect_always_on, FLASH_HZ)
    sfx(rm_define.media_sound_solmization_2C)

def fx_scan():
    # 扫描：纯蓝快闪 + 顶灯跑马 + 扫描音
    leds_set(0, 80, 255, rm_define.effect_flash, 5, top_marquee=True)
    sfx(rm_define.media_sound_scanning)

def fx_scan_pass(pass_n):
    # 每一满行程扫过再提示一次
    leds_set(0, 80, 255, rm_define.effect_flash, 5, top_marquee=True)
    sfx(rm_define.media_sound_scanning)
    log("SCAN pass=%d start" % pass_n)

def fx_recenter():
    # 回中：黄呼吸 + 云台转动音
    leds_set(255, 200, 0, rm_define.effect_breath, FLASH_HZ)
    sfx(rm_define.media_sound_gimbal_rotate)

def fx_lock():
    # 发现人/锁定：紫快闪 + 识别成功
    leds_set(200, 0, 255, rm_define.effect_flash, 6)
    sfx(rm_define.media_sound_recognize_success)

def fx_fire_warn():
    # 示警：橙快闪 + 倒计时
    leds_set(255, 100, 0, rm_define.effect_flash, 7)
    sfx(rm_define.media_sound_count_down)

def fx_fire_burst():
    # 射击：红极快闪 + 枪口灯 + 射击音
    leds_set(255, 0, 0, rm_define.effect_flash, 9, gun_on=True)
    sfx(rm_define.media_sound_shoot)

def fx_person_lost():
    # 丢失目标：橙呼吸 + 被击/告警
    leds_set(255, 140, 0, rm_define.effect_breath, FLASH_HZ)
    sfx(rm_define.media_sound_attacked)

def fx_recover():
    # 找线：白慢闪
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
        extra = " qi=%d/%d pass=%d tgt=%.0f yaw=%.0f stuck=%d person=%s" % (
            g_scan_qi, len(g_scan_queue), g_scan_pass, g_scan_target_yaw,
            get_yaw(), g_scan_stuck, str(has_p)
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
    try:
        ir_blaster_ctrl.stop()
    except Exception:
        pass
    try:
        led_ctrl.gun_led_off()
    except Exception:
        pass

def fire_ir_warn_once():
    """红外/光电示警（优先 ir_blaster；失败则 gun 单发）。"""
    global g_fire_count, g_ir_done
    if g_ir_done:
        log("IR_WARN skip already")
        return
    fx_fire_warn()
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
        gun_ctrl.set_fire_count(1)
        gun_ctrl.fire_once()
        log("IR_WARN fallback gun fire_once")
    g_fire_count = g_fire_count + 1
    g_ir_done = True

def fire_bead_burst_start():
    if ENABLE_FIRE == False:
        log("BURST_ON skip")
        return
    fx_fire_burst()
    gun_ctrl.set_fire_count(1)
    gun_ctrl.fire_continuous()
    log("BURST_ON 1s")

def fire_bead_burst_stop():
    gun_ctrl.stop()
    try:
        led_ctrl.gun_led_off()
    except Exception:
        pass
    log("BURST_OFF 1s")

# =============================================================================
# SCAN：先 PREP 到左限，再两遍满行程扫（每遍 ~490°，覆盖整片 FOV）
# 满扫队列: [+LIM, -LIM]  （从 -LIM 出发）
# =============================================================================
def scan_queue_full():
    return [YAW_LIM, -YAW_LIM]

def scan_save_breakpoint():
    global g_brk_valid, g_brk_target_yaw
    g_brk_valid = True
    g_brk_target_yaw = g_scan_target_yaw
    log("BRK save tgt=%.0f yaw=%.0f qi=%d pass=%d" % (
        g_brk_target_yaw, get_yaw(), g_scan_qi, g_scan_pass
    ))

def scan_queue_after_lost():
    """
    人离开后：
    - 无断点: 完整两遍满扫（带 prep）
    - 正扫中(目标+L): 先到 +L 再 -L
    - 反扫中(目标-L): 先到 -L，再补一遍满扫 +L/-L
    """
    if g_brk_valid == False:
        log("LOST queue: full 2-pass")
        return scan_queue_full(), True
    if g_brk_target_yaw > 0:
        log("LOST queue: finish +L then -L")
        return [YAW_LIM, -YAW_LIM], False
    log("LOST queue: finish -L then full +L/-L")
    return [-YAW_LIM, YAW_LIM, -YAW_LIM], False

def scan_prep_to_left():
    """快速到左限，使第一圈就是满行程（不再从 0 只转半边）。"""
    yaw0 = get_yaw()
    if abs(yaw0 - (-YAW_LIM)) <= YAW_ARRIVE:
        log("SCAN prep skip already left yaw=%.0f" % yaw0)
        return
    set_gimbal_speed(PREP_YAW_SPEED)
    log("SCAN prep -> -LIM yaw0=%.0f" % yaw0)
    try:
        gimbal_ctrl.yaw_ctrl(-YAW_LIM)
    except Exception:
        # 阻塞失败时用速度环兜底
        t0 = now_s()
        while (now_s() - t0) < 2.5:
            if abs(get_yaw() - (-YAW_LIM)) <= YAW_ARRIVE:
                break
            gimbal_ctrl.rotate_with_speed(-SCAN_YAW_SPEED, 0)
            time.sleep(LOOP_DT)
        gimbal_stop()
    time.sleep(0.05)
    log("SCAN prep done yaw=%.0f" % get_yaw())

def scan_load_segment(qi):
    global g_scan_qi, g_scan_target_yaw, g_scan_last_yaw, g_scan_stuck, g_scan_pass
    g_scan_qi = qi
    g_scan_target_yaw = g_scan_queue[qi]
    g_scan_last_yaw = get_yaw()
    g_scan_stuck = 0
    g_scan_pass = qi + 1
    fx_scan_pass(g_scan_pass)
    log("SCAN seg qi=%d tgt_yaw=%.0f yaw0=%.0f" % (qi, g_scan_target_yaw, g_scan_last_yaw))

def scan_start_queue(queue, reason, do_prep):
    global g_scan_queue
    g_scan_queue = queue
    gimbal_set_pitch_scan()
    time.sleep(0.05)
    if do_prep:
        scan_prep_to_left()
    if len(g_scan_queue) <= 0:
        log("SCAN empty queue")
        return
    scan_load_segment(0)
    log("SCAN queue n=%d prep=%s | %s" % (len(g_scan_queue), str(do_prep), reason))

def scan_tick_turn():
    """朝目标绝对 yaw 转动；到位或卡住返回 True。"""
    global g_scan_last_yaw, g_scan_stuck
    yaw = get_yaw()
    err = g_scan_target_yaw - yaw
    if abs(err) <= YAW_ARRIVE:
        log("SCAN arrive yaw=%.0f tgt=%.0f" % (yaw, g_scan_target_yaw))
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

def scan_finish_recenter():
    """高速回中：提高 set_rotate_speed 后 yaw_ctrl(0)，避免默认慢速拖很久。"""
    gimbal_stop()
    fx_recenter()
    yaw0 = get_yaw()
    log("SCAN fast home from yaw=%.0f spd=%.0f" % (yaw0, HOME_YAW_SPEED))
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
    try:
        gimbal_ctrl.pitch_ctrl(PITCH_LINE)
    except Exception:
        pass
    time.sleep(0.1)
    # 若仍偏离，再补一次
    if abs(get_yaw()) > 12.0:
        set_gimbal_speed(HOME_YAW_SPEED)
        try:
            gimbal_ctrl.yaw_ctrl(0)
        except Exception:
            pass
    gimbal_stop()
    log("SCAN home done yaw=%.0f" % get_yaw())

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
        fx_patrol()
        g_patrol_line_t0 = 0.0
        g_line_hit = 0
        g_line_miss = 0
        vision_ctrl.enable_detection(rm_define.vision_detection_line)

    if s == STATE_SCAN:
        fire_stop()
        chassis_halt()
        robot_ctrl.set_mode(rm_define.robot_mode_free)
        fx_scan()
        g_brk_valid = False
        scan_start_queue(scan_queue_full(), "normal_scan", True)

    if s == STATE_LOST_SCAN:
        fire_stop()
        chassis_halt()
        gimbal_stop()
        pid_reset_aim()
        robot_ctrl.set_mode(rm_define.robot_mode_free)
        fx_person_lost()
        time.sleep(0.15)
        q, do_prep = scan_queue_after_lost()
        # 丢失后稍等告警音，再进入扫描灯效
        fx_scan()
        scan_start_queue(q, "lost_scan", do_prep)

    if s == STATE_LOCK:
        if old == STATE_SCAN or old == STATE_LOST_SCAN:
            scan_save_breakpoint()
        fire_stop()
        chassis_halt()
        robot_ctrl.set_mode(rm_define.robot_mode_free)
        gimbal_set_pitch_scan()
        pid_reset_aim()
        fx_lock()
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
        if g_ir_done:
            g_fire_phase = FIRE_PHASE_IR_DONE
            fx_fire_burst()
        else:
            g_fire_phase = FIRE_PHASE_AIM
            fx_lock()
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
        fx_recover()
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
    if line_stable_false():
        g_patrol_line_t0 = 0.0
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
        set_state(STATE_SCAN, "follow_time_up")

def tick_scan_common(is_lost):
    chassis_halt()
    if people_seen():
        gimbal_stop()
        log("SCAN person -> LOCK")
        set_state(STATE_LOCK, "person_on_scan")
        return
    if scan_tick_turn() == False:
        return
    log("SCAN seg done yaw=%.0f tgt=%.0f" % (get_yaw(), g_scan_target_yaw))
    adv = scan_advance_or_finish()
    if adv == "next":
        return
    scan_finish_recenter()
    if is_lost:
        log("LOST_SCAN done -> RECOVER")
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
    log("setup done v1.7.0 lim=%.0f spd=%.0f home=%.0f" % (
        YAW_LIM, SCAN_YAW_SPEED, HOME_YAW_SPEED
    ))

def start():
    global g_state
    print("======== Line Guard start ========")
    print("# LINE_GUARD_VERSION=1.7.0 stamp=2026-08-03 15:38:49")
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
