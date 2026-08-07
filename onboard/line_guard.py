# LINE_GUARD_VERSION=1.38.1 stamp=2026-08-07 13:45:00  (paste this whole file; check stamp matches latest)
# -*- coding: utf-8 -*-
# S1 Line Guard — 单文件粘贴进 App 实验室
#
# 权威行为规格：../docs/behavior-spec.md（未写入规格的限制禁止实现）
# 摘要：hit≥3 → 立即告警+开火；FIRE 内不因丢人不停射；首段 3s 射完后 miss>3 才重扫

# =============================================================================
# CONFIG — 可调参数
# =============================================================================
T_MOVE = 6.0                    # PATROL 贴线前进多久（秒）后进入 SCAN
PERSON_HIT_NEED = 3             # 连续 hit≥此值才算发现 → 立即 FIRE
PERSON_MISS_NEED = 3            # 已射满至少 3s 后：连续 miss 超过此值 → 整圈 SCAN

PERSON_FIRE_MIN_W = 0.10        # 开火框最小宽（记忆框尺寸也用此）
PERSON_FIRE_MIN_H = 0.16

PITCH_LINE = -20                # 巡线低头绝对角
PITCH_SCAN = 20                 # 扫人/交战抬头
GIMBAL_YAW_SPEED = 540.0

SCAN_CW = 180.0
SCAN_CCW = -180.0
SCAN_STEP_DEG = 45.0
SCAN_LOOK_OPS = 5               # 每角查人次数；满仍 hit<3 → 下一角

# 巡线：err=cx-0.5 纯 P + 近中心软增益
LINE_SPEED = 0.12
LINE_YAW_KP = 140.0
LINE_YAW_MAX = 55.0
LINE_SOFT_ERR = 0.08
LINE_SOFT_GAIN = 0.45
LINE_CONFIRM_FRAMES = 3
LINE_LOST_S = 1.5
LINE_LOG_DT = 1.0

# 瞄准 PID：远距快锁、近中心软刹
AIM_YAW_KP = 95.0
AIM_YAW_KI = 6.0
AIM_YAW_KD = 38.0
AIM_YAW_OUT_MAX = 130.0
AIM_PITCH_KP = 60.0
AIM_PITCH_KI = 5.0
AIM_PITCH_KD = 30.0
AIM_PITCH_OUT_MAX = 42.0
AIM_DEADZONE = 0.025
AIM_OK_ERR = 0.07
AIM_SOFT_ERR = 0.12
AIM_SOFT_MIN = 0.22
AIM_ACQUIRE_ERR = 0.16
AIM_ACQUIRE_YAW_MAX = 300.0
AIM_D_MAX = 1.8
AIM_TRACK_ALPHA = 0.48

PERSON_MIN_W = 0.08
PERSON_MIN_H = 0.14
PERSON_MIN_ASPECT = 1.15
PERSON_MAX_CY = 0.72

T_FIRE_ON = 3.0                 # 射击段：边瞄边射
T_FIRE_OFF = 3.0                # 停火段：只瞄
FIRE_PULSE_INTERVAL = 0.18
FIRE_BEADS_PER_PULSE = 2

ENABLE_FIRE = True
FLASH_HZ = 3

# =============================================================================
# STATE — 状态机与全局变量
# =============================================================================
STATE_INIT = 0
STATE_PATROL = 1
STATE_SCAN = 2
STATE_FIRE = 3

# FIRE 内节奏：射 3s ↔ 停火只瞄 3s（见 behavior-spec）
FIRE_PHASE_SHOOT = 0            # 边瞄边射
FIRE_PHASE_HOLD = 1             # 只瞄不射

g_state = STATE_INIT
g_state_t0 = 0.0
g_person_miss = 0
g_person_hit = 0
g_patrol_line_t0 = 0.0
g_aim_t_prev = 0.0
g_fire_phase = FIRE_PHASE_SHOOT
g_phase_t0 = 0.0
g_ir_done = False
g_last_shot_t = 0.0
g_burst_shots = 0
g_iy = 0.0
g_ey_prev = 0.0
g_ip = 0.0
g_ep_prev = 0.0
g_line_hit = 0
g_line_miss = 0
g_line_cx = 0.5
g_line_err = 0.0
g_line_yaw_spd = 0.0
g_line_pts = 0
g_line_ever_ok = False
g_line_log_t = 0.0
g_line_miss_t0 = 0.0
g_track_on = False
g_track_x = 0.5
g_track_y = 0.5
g_track_w = 0.2
g_track_h = 0.3
g_min_fire_done = False
g_last_see_ok = False
g_last_see_x = 0.5
g_last_see_y = 0.5
g_last_see_w = 0.2
g_last_see_h = 0.3

# SCAN
g_scan_queue = []
g_scan_qi = 0
g_scan_target_yaw = 0.0
g_scan_planned_yaw = 0.0
g_scan_waypoints = []
g_scan_wi = 0
g_scan_seg_name = ""
g_scan_step_i = 0
g_scan_look_ops = 0
g_scan_cycle_look_ops = 0
g_scan_cycle_turns = 0
g_scan_t_after_turn = 0.0
g_last_person_reject_t = 0.0

# =============================================================================
# LOG / MATH — 时间、日志、限幅、PID 步进
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
    if s == STATE_FIRE:
        return "FIRE"
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

def get_pitch():
    return gimbal_ctrl.get_axis_angle(rm_define.gimbal_axis_pitch)

def pid_reset_aim():
    global g_iy, g_ey_prev, g_ip, g_ep_prev
    g_iy = 0.0
    g_ey_prev = 0.0
    g_ip = 0.0
    g_ep_prev = 0.0

def pid_step(err, i_acc, e_prev, kp, ki, kd, out_max, dt):
    """瞄准用 PID 一步。死区内清 I；D 限幅。"""
    if abs(err) < AIM_DEADZONE:
        return 0.0, 0.0, err
    i_new = i_acc + err * dt
    i_new = clamp(i_new, -0.6, 0.6)
    if dt > 0.0001:
        d = (err - e_prev) / dt
    else:
        d = 0.0
    d = clamp(d, -AIM_D_MAX, AIM_D_MAX)
    out = kp * err + ki * i_new + kd * d
    out = clamp(out, -out_max, out_max)
    if (out >= out_max and err > 0) or (out <= -out_max and err < 0):
        i_new = i_acc
    return out, i_new, err

# =============================================================================
# 灯光 + 内置音效 — 各状态视觉/听觉反馈
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

# -----------------------------------------------------------------------------
# 机身灯光含义（测试时对照）
#
#  | 何时              | 颜色观感     | 效果              | 枪口灯 |
#  | PATROL 巡线       | 绿常亮       | always_on         | 关     |
#  | SCAN 扫人/换段    | 蓝闪 + 顶跑马 | flash + marquee  | 关     |
#  | SCAN 段回中(≈0°)  | 黄呼吸       | breath            | 关     |
#  | 发现人 / IR / 停火瞄 | 红闪      | flash             | 关     |
#  | FIRE 射击段(射3s) | 红闪         | flash 很快        | 开     |
#  | 确认丢人→重扫前   | 紫闪         | flash             | 关     |
#
# 红闪无枪口=交战未射；红闪+枪口=正在射；紫闪=丢人；绿=巡线。
# -----------------------------------------------------------------------------
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
    """绿灯常亮 = PATROL 巡线中。"""
    leds_set(0, 255, 0, rm_define.effect_always_on, FLASH_HZ)
    sfx(rm_define.media_sound_solmization_2C)

def fx_scan():
    """蓝闪+顶跑马 = SCAN 扫人开始/进行中。"""
    leds_set(0, 80, 255, rm_define.effect_flash, 5, top_marquee=True)
    sfx(rm_define.media_sound_scanning)

def fx_scan_seg(seg_name):
    """蓝闪+顶跑马 = SCAN 进入某一大段（与 fx_scan 同色）。"""
    leds_set(0, 80, 255, rm_define.effect_flash, 5, top_marquee=True)
    sfx(rm_define.media_sound_scanning)
    log("SCAN seg start: %s" % seg_name)

def fx_recenter():
    """黄呼吸 = 云台回中/扫到 0° 段。"""
    leds_set(255, 200, 0, rm_define.effect_breath, FLASH_HZ)
    sfx(rm_define.media_sound_gimbal_rotate)

def fx_combat():
    """红闪 = 交战（进 FIRE / 报警）。"""
    leds_set(255, 0, 0, rm_define.effect_flash, 6)
    sfx(rm_define.media_sound_recognize_success)

def fx_fire_ir_led():
    """红闪 = 红外示警（无枪口灯）。"""
    leds_set(255, 0, 0, rm_define.effect_flash, 7)

def fx_fire_burst_led():
    """红闪+枪口灯 = FIRE 射击段。"""
    leds_set(255, 0, 0, rm_define.effect_flash, 8, gun_on=True)

def fx_fire_wait_led():
    """红闪无枪口 = FIRE 停火只瞄。"""
    leds_set(255, 0, 0, rm_define.effect_flash, 6)

def fx_person_lost():
    """紫闪 = 确认丢人，即将整圈重扫。"""
    leds_set(200, 0, 255, rm_define.effect_flash, 5)
    sfx(rm_define.media_sound_attacked)

# =============================================================================
# VISION — 行人检测与防抖
# =============================================================================
def people_reject_log(reason, x, y, w, h):
    """API 报了人但几何不像行人时记一条（节流，避免刷屏）。"""
    global g_last_person_reject_t
    t = now_s()
    if g_last_person_reject_t > 0.0 and (t - g_last_person_reject_t) < 0.4:
        return
    g_last_person_reject_t = t
    log("PERSON reject %s xy=(%.2f,%.2f) wh=(%.2f,%.2f) asp=%.2f" % (
        reason, x, y, w, h, (h / w) if w > 0.001 else 0.0
    ))

def people_get_first():
    """
    取第一个行人框。S1 常对行李/地面误报 n>=1，故除计数外还过滤：
      最小宽高、高宽比（站立人偏高）、框中心不能过靠画面下方。
    返回 (ok, x, y, w, h)
    """
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
    # 以下：API 认为「有人」，但可能是误检
    try:
        wf = float(w)
        hf = float(h)
        xf = float(x)
        yf = float(y)
    except Exception:
        return False, 0.5, 0.5, 0.0, 0.0
    if xf < 0.0 or xf > 1.0 or yf < 0.0 or yf > 1.0:
        people_reject_log("xy_range", xf, yf, wf, hf)
        return False, xf, yf, wf, hf
    if wf < PERSON_MIN_W or hf < PERSON_MIN_H:
        people_reject_log("size", xf, yf, wf, hf)
        return False, xf, yf, wf, hf
    if wf > 0.001 and (hf / wf) < PERSON_MIN_ASPECT:
        people_reject_log("aspect", xf, yf, wf, hf)
        return False, xf, yf, wf, hf
    if yf > PERSON_MAX_CY:
        people_reject_log("too_low", xf, yf, wf, hf)
        return False, xf, yf, wf, hf
    return True, xf, yf, wf, hf

def person_hit_reset():
    global g_person_hit
    g_person_hit = 0

def person_hit_update(need):
    """连续 need 帧有效检出 → 确认发现人。"""
    global g_person_hit
    if need < 1:
        need = 1
    ok, x, y, w, h = people_get_first()
    if ok:
        g_person_hit = g_person_hit + 1
    else:
        g_person_hit = 0
    return g_person_hit >= need


def track_clear():
    global g_track_on, g_track_x, g_track_y, g_track_w, g_track_h
    g_track_on = False
    g_track_x = 0.5
    g_track_y = 0.5
    g_track_w = 0.2
    g_track_h = 0.3

def track_set(x, y, w, h):
    """建立/刷新锁定记忆框；已在跟踪时 EMA，首锁直接跳到当前框。"""
    global g_track_on, g_track_x, g_track_y, g_track_w, g_track_h
    if g_track_on:
        a = AIM_TRACK_ALPHA
        g_track_x = a * x + (1.0 - a) * g_track_x
        g_track_y = a * y + (1.0 - a) * g_track_y
        g_track_w = a * w + (1.0 - a) * g_track_w
        g_track_h = a * h + (1.0 - a) * g_track_h
    else:
        g_track_x = x
        g_track_y = y
        g_track_w = w
        g_track_h = h
    g_track_on = True

def last_see_set(x, y, w, h):
    global g_last_see_ok, g_last_see_x, g_last_see_y, g_last_see_w, g_last_see_h
    g_last_see_ok = True
    g_last_see_x = x
    g_last_see_y = y
    g_last_see_w = w
    g_last_see_h = h

def track_from_people():
    """若本帧有效检出则刷新记忆框，返回是否刷新。"""
    ok, x, y, w, h = people_get_first()
    if ok:
        track_set(x, y, w, h)
        last_see_set(x, y, w, h)
        return True
    return False

def track_ensure_for_engage():
    """
    发现后必须有记忆框：当前帧 → 扫描末次有效框 → 默认尺寸中心框。
    保证进入 FIRE 后可以告警/开火，不再卡在「no solid person」。
    """
    if track_from_people():
        return "live"
    if g_last_see_ok:
        track_set(g_last_see_x, g_last_see_y, g_last_see_w, g_last_see_h)
        return "last_see"
    track_set(0.5, 0.5, PERSON_FIRE_MIN_W, PERSON_FIRE_MIN_H)
    return "default"

def person_track_update():
    """FIRE：有检出刷新记忆并清 miss；无检出 miss+1，记忆框保留、位置不撤。"""
    global g_person_miss
    if track_from_people():
        g_person_miss = 0
        return True
    g_person_miss = g_person_miss + 1
    return False

def person_confirmed_lost():
    """仅首段 3s 射击完成后，连续 miss 超过门槛才允许放弃。"""
    if g_min_fire_done == False:
        return False
    return g_person_miss > PERSON_MISS_NEED

def leave_combat_to_rescan(reason):
    """结束交战：停火、清记忆，整圈 SCAN。"""
    global g_min_fire_done, g_last_see_ok
    fire_stop()
    person_hit_reset()
    track_clear()
    g_last_see_ok = False
    g_min_fire_done = False
    pid_reset_aim()
    fx_person_lost()
    set_state(STATE_SCAN, "rescan_after_%s" % reason)

def engage_fire_immediate(reason):
    """发现人后立即进入 FIRE：告警 + 开火（不再回查「是否还有人」才开火）。"""
    src = track_ensure_for_engage()
    log("ENGAGE immediate FIRE | %s track=(%.2f,%.2f) src=%s" % (
        reason, g_track_x, g_track_y, src
    ))
    set_state(STATE_FIRE, reason)

def aim_pid_dt():
    """PID 微分用实际步间隔（非 sleep）；首帧用 0.05。"""
    global g_aim_t_prev
    t = now_s()
    if g_aim_t_prev <= 0.0:
        dt = 0.05
    else:
        dt = t - g_aim_t_prev
        if dt < 0.001:
            dt = 0.001
        if dt > 0.5:
            dt = 0.5
    g_aim_t_prev = t
    return dt

def _aim_soft_scale(abs_err):
    """近中心软刹：|err| 从 DEADZONE→SOFT_ERR 时输出上限从 SOFT_MIN→1。"""
    if abs_err >= AIM_SOFT_ERR:
        return 1.0
    span = AIM_SOFT_ERR - AIM_DEADZONE
    if span < 0.001:
        return AIM_SOFT_MIN
    t = (abs_err - AIM_DEADZONE) / span
    if t < 0.0:
        t = 0.0
    if t > 1.0:
        t = 1.0
    t = t ** 0.5
    return AIM_SOFT_MIN + (1.0 - AIM_SOFT_MIN) * t

def aim_pid_towards_xy(x, y, dt):
    """瞄准：大偏差快锁；软刹区降 max 防过冲；仅双轴死区硬停。"""
    global g_iy, g_ey_prev, g_ip, g_ep_prev
    err_yaw = x - 0.5
    err_pitch = y - 0.5
    abs_ey = abs(err_yaw)
    abs_ep = abs(err_pitch)
    if abs_ey < AIM_DEADZONE and abs_ep < AIM_DEADZONE:
        g_iy = 0.0
        g_ip = 0.0
        g_ey_prev = err_yaw
        g_ep_prev = err_pitch
        gimbal_stop()
        return True
    if abs_ey >= AIM_ACQUIRE_ERR:
        yaw_max = AIM_ACQUIRE_YAW_MAX
    else:
        yaw_max = AIM_YAW_OUT_MAX * _aim_soft_scale(abs_ey)
    pitch_max = AIM_PITCH_OUT_MAX * _aim_soft_scale(abs_ep)
    yaw_spd, g_iy, g_ey_prev = pid_step(
        err_yaw, g_iy, g_ey_prev, AIM_YAW_KP, AIM_YAW_KI, AIM_YAW_KD, yaw_max, dt
    )
    pitch_spd, g_ip, g_ep_prev = pid_step(
        err_pitch, g_ip, g_ep_prev, AIM_PITCH_KP, AIM_PITCH_KI, AIM_PITCH_KD, pitch_max, dt
    )
    try:
        gimbal_ctrl.rotate_with_speed(yaw_spd, -pitch_spd)
    except Exception:
        pass
    return abs_ey < AIM_OK_ERR and abs_ep < AIM_OK_ERR

def aim_pid_track():
    """有检出 EMA 刷新并跟平滑中心；无检出跟记忆框。"""
    dt = aim_pid_dt()
    ok, x, y, w, h = people_get_first()
    if ok:
        track_set(x, y, w, h)
        return aim_pid_towards_xy(g_track_x, g_track_y, dt), True
    if g_track_on:
        return aim_pid_towards_xy(g_track_x, g_track_y, dt), False
    gimbal_stop()
    return False, False

# =============================================================================
# LINE VISION — RmList 读蓝线
# =============================================================================
def line_get_rmlist():
    raw = vision_ctrl.get_line_detection_info()
    try:
        return RmList(raw)
    except Exception:
        return raw

def line_read():
    """官方：len==42 且 [2]>=1，cx=[19]。返回 (ok, cx, n, pts)。"""
    info = line_get_rmlist()
    try:
        n = len(info)
    except Exception:
        return False, 0.5, 0, 0
    pts = 0
    try:
        pts = int(info[2])
    except Exception:
        pts = 0
    # 与官方一致优先 len==42；兼容少数固件 >=42
    if pts >= 1 and (n == 42 or n >= 42):
        try:
            cx = float(info[19])
            return True, cx, n, pts
        except Exception:
            return False, 0.5, n, pts
    return False, 0.5, n, pts

def line_update():
    global g_line_hit, g_line_miss, g_line_cx, g_line_pts, g_line_ever_ok, g_line_miss_t0
    ok, cx, n, pts = line_read()
    g_line_pts = pts
    if ok:
        g_line_cx = cx
        g_line_hit = g_line_hit + 1
        g_line_miss = 0
        g_line_miss_t0 = 0.0
        g_line_ever_ok = True
    else:
        g_line_miss = g_line_miss + 1
        g_line_hit = 0
        if g_line_miss_t0 <= 0.0:
            g_line_miss_t0 = now_s()

def line_stable_true():
    return g_line_hit >= LINE_CONFIRM_FRAMES

def line_stable_false():
    """连续 LINE_LOST_S 秒看不到线才判丢线。"""
    if g_line_miss_t0 <= 0.0:
        return False
    return (now_s() - g_line_miss_t0) >= LINE_LOST_S

def line_look_down():
    """巡线低头。"""
    down_deg = -PITCH_LINE
    if down_deg < 0:
        down_deg = -down_deg
    try:
        gimbal_ctrl.rotate_with_degree(rm_define.gimbal_down, down_deg)
        log("LINE look_down %d deg" % down_deg)
    except Exception:
        try:
            gimbal_ctrl.pitch_ctrl(PITCH_LINE)
            log("LINE look_down fallback pitch=%d" % PITCH_LINE)
        except Exception:
            log("LINE look_down FAIL")

# =============================================================================
# ACTUATORS — 云台姿态、模式、循线步进、射击
# =============================================================================
def gimbal_set_pitch_scan():
    set_gimbal_speed(GIMBAL_YAW_SPEED)
    try:
        gimbal_ctrl.pitch_ctrl(PITCH_SCAN)
    except Exception:
        pass

def gimbal_ensure_pitch_scan_soft():
    """
    进瞄准前：俯仰偏离扫描角较多才 pitch_ctrl。
    避免阻塞抬头打断 yaw 瞄准。
    """
    try:
        p = get_pitch()
        if abs(p - PITCH_SCAN) <= 6.0:
            return
    except Exception:
        pass
    set_gimbal_speed(GIMBAL_YAW_SPEED)
    try:
        gimbal_ctrl.pitch_ctrl(PITCH_SCAN)
    except Exception:
        pass

def gimbal_pose_line():
    """巡线姿态：yaw=0，pitch 低头。"""
    set_gimbal_speed(GIMBAL_YAW_SPEED)
    try:
        gimbal_ctrl.angle_ctrl(0, PITCH_LINE)
    except Exception:
        try:
            gimbal_ctrl.yaw_ctrl(0)
        except Exception:
            pass
        try:
            gimbal_ctrl.pitch_ctrl(PITCH_LINE)
        except Exception:
            pass
    try:
        gimbal_ctrl.pitch_ctrl(PITCH_LINE)
    except Exception:
        pass
    log("POSE line yaw=%.0f pitch=%.0f" % (get_yaw(), get_pitch()))

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

def mode_ensure_free(reason):
    """SCAN/FIRE 等：停底盘 + robot_mode_free，云台独立。"""
    chassis_halt()
    try:
        gimbal_ctrl.rotate_with_speed(0, 0)
    except Exception:
        pass
    try:
        gimbal_ctrl.stop()
    except Exception:
        pass
    robot_ctrl.set_mode(rm_define.robot_mode_free)
    log("MODE free | %s" % reason)

def mode_ensure_line_follow(reason):
    """PATROL：chassis_follow，云台带动底盘循线。"""
    robot_ctrl.set_mode(rm_define.robot_mode_chassis_follow)
    log("MODE chassis_follow | %s" % reason)

def line_follow_step():
    """贴线：err=cx-0.5 → P → yaw（限幅+近中心软增益）；固定 LINE_SPEED。"""
    global g_line_err, g_line_yaw_spd
    cx = g_line_cx
    err = cx - 0.5
    g_line_err = err
    abs_e = abs(err)
    yaw_spd = err * LINE_YAW_KP
    if abs_e < LINE_SOFT_ERR and abs_e > 0.001:
        yaw_spd = yaw_spd * LINE_SOFT_GAIN
    if yaw_spd > LINE_YAW_MAX:
        yaw_spd = LINE_YAW_MAX
    if yaw_spd < -LINE_YAW_MAX:
        yaw_spd = -LINE_YAW_MAX
    g_line_yaw_spd = yaw_spd
    try:
        gimbal_ctrl.rotate_with_speed(yaw_spd, 0)
    except Exception:
        pass
    try:
        chassis_ctrl.set_trans_speed(LINE_SPEED)
        chassis_ctrl.move(0)
    except Exception:
        try:
            chassis_ctrl.move_with_speed(LINE_SPEED, 0, 0)
        except Exception:
            chassis_halt()

def line_follow_log_snapshot(tag):
    """关键时刻：start / done / LOST / run。"""
    age = 0.0
    if g_patrol_line_t0 > 0.0:
        age = now_s() - g_patrol_line_t0
    log(
        "PATROL %s cx=%.3f err=%+.3f yaw=%+.0f spd=%.2f pts=%d hit=%d miss=%d age=%.2f"
        % (
            tag,
            g_line_cx,
            g_line_err,
            g_line_yaw_spd,
            LINE_SPEED,
            g_line_pts,
            g_line_hit,
            g_line_miss,
            age,
        )
    )

def line_follow_log_tick():
    """贴线推进时低频过程日志。"""
    global g_line_log_t
    t = now_s()
    if g_line_log_t > 0.0 and (t - g_line_log_t) < LINE_LOG_DT:
        return
    g_line_log_t = t
    line_follow_log_snapshot("run")

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
    """红外示警一次。已进 FIRE 即执行，不再查「当前是否有人」。"""
    global g_ir_done
    if g_ir_done:
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
        log("IR_WARN ok")
    except Exception:
        ok_ir = False
    if ok_ir == False:
        try:
            gun_ctrl.set_fire_count(1)
            gun_ctrl.fire_once()
            log("IR_WARN fallback gun")
        except Exception:
            log("IR_WARN fail")
    g_ir_done = True

def fire_bead_burst_start():
    """射击段开始：已进 FIRE 即脉冲开火，不因丢人跳过。"""
    global g_last_shot_t, g_burst_shots
    fx_fire_burst_led()
    g_last_shot_t = 0.0
    g_burst_shots = 0
    if ENABLE_FIRE == False:
        log("FIRE_ON skip ENABLE_FIRE=0 (led only)")
        return
    fire_bead_pulse_once()
    log("FIRE_ON pulse interval=%.2fs for %.1fs" % (FIRE_PULSE_INTERVAL, T_FIRE_ON))

def fire_bead_pulse_once():
    global g_last_shot_t, g_burst_shots
    if ENABLE_FIRE == False:
        return
    n = FIRE_BEADS_PER_PULSE
    if n < 1:
        n = 1
    if n > 8:
        n = 8
    try:
        gun_ctrl.set_fire_count(n)
        gun_ctrl.fire_once()
        g_burst_shots = g_burst_shots + 1
        g_last_shot_t = now_s()
        log("FIRE pulse #%d" % g_burst_shots)
    except Exception:
        log("FIRE pulse fail")

def fire_bead_burst_tick():
    """射击段内按间隔补发；不因当前帧无人而停射。"""
    if ENABLE_FIRE == False:
        return
    if g_last_shot_t <= 0.0:
        fire_bead_pulse_once()
        return
    if (now_s() - g_last_shot_t) >= FIRE_PULSE_INTERVAL:
        fire_bead_pulse_once()

def fire_bead_burst_stop():
    try:
        gun_ctrl.stop()
    except Exception:
        pass
    try:
        led_ctrl.gun_led_off()
    except Exception:
        pass
    log("FIRE_OFF stop shots=%d" % g_burst_shots)

# =============================================================================
# SCAN — 45° 步进 yaw_ctrl，极限转速，步间查人
# 几何：回中 → +180 → 0 → -180 → 0
# =============================================================================
def scan_queue_full_two_rounds():
    return [SCAN_CW, 0.0, SCAN_CCW, 0.0]

def scan_seg_label(qi, target):
    if qi == 0:
        return "CW_to_%+.0f" % target
    if qi == 1:
        return "back_to_0"
    if qi == 2:
        return "CCW_to_%+.0f" % target
    if qi == 3:
        return "back_to_0"
    return "qi%d_to_%+.0f" % (qi, target)

def scan_yaw_abs(tgt, reason):
    """设定转速后绝对 yaw_ctrl 到 tgt。返回 (yaw0, yaw1, dt_s)。"""
    gimbal_stop()
    set_gimbal_speed(GIMBAL_YAW_SPEED)
    y0 = get_yaw()
    t0 = now_s()
    try:
        gimbal_ctrl.yaw_ctrl(tgt)
    except Exception:
        log("SCAN yaw_ctrl FAIL tgt=%.0f" % tgt)
    y1 = get_yaw()
    dt = now_s() - t0
    gimbal_stop()
    log("SCAN yaw_ctrl %.0f -> %.0f (got %.0f) spd=%.0f dt=%.3f | %s" % (
        y0, tgt, y1, GIMBAL_YAW_SPEED, dt, reason
    ))
    return y0, y1, dt

def gimbal_fast_home(reason, keep_scan_pitch=False):
    """快速回中（绝对角）。"""
    gimbal_stop()
    fx_recenter()
    yaw0 = get_yaw()
    pit0 = 0.0
    try:
        pit0 = get_pitch()
    except Exception:
        pit0 = 0.0
    log("HOME begin yaw=%.0f pitch=%.0f | %s" % (yaw0, pit0, reason))
    set_gimbal_speed(GIMBAL_YAW_SPEED)
    if keep_scan_pitch:
        try:
            gimbal_ctrl.angle_ctrl(0, PITCH_SCAN)
        except Exception:
            try:
                gimbal_ctrl.yaw_ctrl(0)
            except Exception:
                pass
            gimbal_set_pitch_scan()
        gimbal_set_pitch_scan()
    else:
        gimbal_pose_line()
    gimbal_stop()
    log("HOME done yaw=%.0f pitch=%.0f" % (get_yaw(), get_pitch()))

def scan_build_waypoints(y0, y1, step_deg):
    """
    从规划角 y0 到 y1 生成绝对角航点列表（不含 y0，含 y1）。
    步长 step_deg；不读实测 yaw。
    """
    pts = []
    dy = y1 - y0
    if abs(dy) < 0.01:
        return pts
    step = abs(step_deg)
    if step < 5.0:
        step = 5.0
    if dy > 0:
        y = y0 + step
        while y < y1 - 0.01:
            pts.append(y)
            y = y + step
        pts.append(y1)
    else:
        y = y0 - step
        while y > y1 + 0.01:
            pts.append(y)
            y = y - step
        pts.append(y1)
    return pts

def scan_load_segment(qi):
    """装载第 qi 段：规划角 g_scan_planned_yaw → 大目标，生成航点。"""
    global g_scan_qi, g_scan_target_yaw, g_scan_seg_name
    global g_scan_waypoints, g_scan_wi
    g_scan_qi = qi
    g_scan_target_yaw = g_scan_queue[qi]
    g_scan_seg_name = scan_seg_label(qi, g_scan_target_yaw)
    g_scan_waypoints = scan_build_waypoints(
        g_scan_planned_yaw, g_scan_target_yaw, SCAN_STEP_DEG
    )
    g_scan_wi = 0
    if abs(g_scan_target_yaw) < 0.1:
        fx_recenter()
    else:
        fx_scan_seg(g_scan_seg_name)
    log(
        "SCAN seg qi=%d %s plan=%.0f -> %.0f n_wp=%d %s"
        % (
            qi, g_scan_seg_name, g_scan_planned_yaw, g_scan_target_yaw,
            len(g_scan_waypoints), str(g_scan_waypoints)
        )
    )

def scan_diag_reset_cycle():
    """新 SCAN 循环：清诊断计数与规划角。"""
    global g_scan_step_i, g_scan_look_ops, g_scan_cycle_look_ops
    global g_scan_cycle_turns, g_scan_t_after_turn, g_scan_planned_yaw
    global g_scan_waypoints, g_scan_wi
    g_scan_step_i = 0
    g_scan_look_ops = 0
    g_scan_cycle_look_ops = 0
    g_scan_cycle_turns = 0
    g_scan_t_after_turn = now_s()
    g_scan_planned_yaw = 0.0
    g_scan_waypoints = []
    g_scan_wi = 0

def scan_start_full(reason):
    """从回中开始完整 SCAN 循环。"""
    global g_scan_queue, g_scan_planned_yaw
    mode_ensure_free("scan_start_full")
    g_scan_queue = scan_queue_full_two_rounds()
    person_hit_reset()
    scan_diag_reset_cycle()
    set_gimbal_speed(GIMBAL_YAW_SPEED)
    try:
        gimbal_ctrl.angle_ctrl(0, PITCH_SCAN)
    except Exception:
        try:
            gimbal_ctrl.yaw_ctrl(0)
        except Exception:
            pass
        gimbal_set_pitch_scan()
    gimbal_set_pitch_scan()
    g_scan_planned_yaw = 0.0
    g_scan_t_after_turn = now_s()
    log("SCAN home first plan_yaw=0")
    if len(g_scan_queue) <= 0:
        log("SCAN empty queue")
        return
    fx_scan()
    scan_load_segment(0)
    log(
        "SCAN plan: home -> CW%+.0f -> 0 -> CCW%+.0f -> 0 | step=%.0f look=%d hit_need=%d | %s"
        % (SCAN_CW, SCAN_CCW, SCAN_STEP_DEG, SCAN_LOOK_OPS, PERSON_HIT_NEED, reason)
    )

def scan_look_once():
    """
    一次查人采样，更新连续 hit。
    仅 hit≥PERSON_HIT_NEED 时 locked=True（才算发现人）。
    返回 (found, frame_hit, dt_op)；frame_hit=本帧有框（未达 3 不算发现）。
    """
    global g_scan_look_ops, g_scan_cycle_look_ops
    t0 = now_s()
    found = person_hit_update(PERSON_HIT_NEED)
    dt = now_s() - t0
    frame_hit = g_person_hit > 0
    g_scan_look_ops = g_scan_look_ops + 1
    g_scan_cycle_look_ops = g_scan_cycle_look_ops + 1
    gap = 0.0
    if g_scan_t_after_turn > 0.0:
        gap = t0 - g_scan_t_after_turn
    whs = ""
    if frame_hit:
        ok2, px, py, pw, ph = people_get_first()
        if ok2:
            last_see_set(px, py, pw, ph)
            whs = " xy=(%.2f,%.2f) wh=(%.2f,%.2f)" % (px, py, pw, ph)
    log(
        "SCAN_LOOK step=%d yaw=%.0f seg=%s look_ops=%d hit=%d/%d frame=%s "
        "found=%s dt_op=%.3f gap=%.3f%s"
        % (
            g_scan_step_i, get_yaw(), g_scan_seg_name, g_scan_look_ops,
            g_person_hit, PERSON_HIT_NEED, str(frame_hit), str(found), dt, gap, whs
        )
    )
    return found, frame_hit, dt

def scan_tick_turn():
    """
    按规划航点执行一步绝对 yaw_ctrl（S1 阻塞到位）；不读实测 yaw 判定。
    返回 (seg_done, did_turn)
    """
    global g_scan_step_i, g_scan_look_ops, g_scan_cycle_turns, g_scan_t_after_turn
    global g_scan_wi, g_scan_planned_yaw
    if g_scan_wi >= len(g_scan_waypoints):
        log(
            "SCAN_SEG_DONE step=%d plan=%.0f tgt=%.0f look_ops=%d | %s"
            % (g_scan_step_i, g_scan_planned_yaw, g_scan_target_yaw, g_scan_look_ops, g_scan_seg_name)
        )
        return True, False
    nxt = g_scan_waypoints[g_scan_wi]
    g_scan_wi = g_scan_wi + 1
    look_before = g_scan_look_ops
    y0, y1, dt_turn = scan_yaw_abs(nxt, "%s wp" % g_scan_seg_name)
    g_scan_planned_yaw = nxt
    g_scan_cycle_turns = g_scan_cycle_turns + 1
    g_scan_step_i = g_scan_step_i + 1
    g_scan_t_after_turn = now_s()
    g_scan_look_ops = 0
    seg_done = g_scan_wi >= len(g_scan_waypoints)
    log(
        "SCAN_TURN step=%d plan_cmd=%.0f look_ops_before=%d dt_turn=%.3f "
        "wp=%d/%d seg_done=%s | %s"
        % (
            g_scan_step_i, nxt, look_before, dt_turn,
            g_scan_wi, len(g_scan_waypoints), str(seg_done), g_scan_seg_name
        )
    )
    return seg_done, True

def scan_advance_or_finish():
    """本段完成：下一段或整圈结束。"""
    global g_scan_qi, g_scan_look_ops
    gimbal_stop()
    g_scan_look_ops = 0
    ni = g_scan_qi + 1
    if ni < len(g_scan_queue):
        scan_load_segment(ni)
        return "next"
    return "done"

def scan_log_cycle_summary(where):
    """整圈结束诊断摘要。"""
    log(
        "SCAN_CYCLE_SUM where=%s steps=%d turns=%d look_ops_total=%d "
        "qi=%d/%d yaw=%.0f"
        % (
            where, g_scan_step_i, g_scan_cycle_turns, g_scan_cycle_look_ops,
            g_scan_qi, len(g_scan_queue), get_yaw()
        )
    )

# =============================================================================
# STATE MACHINE — 入口与各态 tick
# =============================================================================
def set_state(s, reason):
    """状态切换：非 PATROL → free；PATROL → chassis_follow + 低头循线。"""
    global g_state, g_state_t0, g_person_miss
    global g_patrol_line_t0, g_fire_phase, g_phase_t0
    global g_ir_done, g_line_hit, g_line_miss, g_line_ever_ok
    global g_min_fire_done, g_burst_shots, g_last_shot_t
    global g_line_cx, g_line_err, g_line_yaw_spd, g_line_log_t, g_line_miss_t0
    old = g_state
    g_state = s
    g_state_t0 = now_s()
    if s != STATE_FIRE:
        g_person_miss = 0
    log("STATE %s -> %s | %s" % (state_name(old), state_name(s), reason))

    if s != STATE_PATROL:
        fire_stop()
        mode_ensure_free("enter_%s" % state_name(s))

    if s == STATE_PATROL:
        fire_stop()
        chassis_halt()
        mode_ensure_line_follow("patrol_start")
        line_look_down()
        fx_patrol()
        g_patrol_line_t0 = 0.0
        g_line_hit = 0
        g_line_miss = 0
        g_line_miss_t0 = 0.0
        g_line_cx = 0.5
        g_line_err = 0.0
        g_line_yaw_spd = 0.0
        g_line_log_t = 0.0
        g_line_ever_ok = False
        person_hit_reset()
        vision_ctrl.enable_detection(rm_define.vision_detection_line)
        vision_ctrl.line_follow_color_set(rm_define.line_follow_color_blue)
        log("PATROL ready spd=%.2f yaw_kp=%.0f max=%.0f" % (
            LINE_SPEED, LINE_YAW_KP, LINE_YAW_MAX
        ))

    # SCAN：完整两遍步进扫描
    if s == STATE_SCAN:
        scan_start_full("normal_scan")



    # FIRE：告警 + 开火；不因当前无人而跳过
    if s == STATE_FIRE:
        mode_ensure_free("enter_FIRE")
        # 抬头到扫人俯仰；不 yaw 回中，保持发现时朝向
        gimbal_ensure_pitch_scan_soft()
        pid_reset_aim()
        fx_combat()
        g_person_miss = 0
        g_min_fire_done = False
        g_burst_shots = 0
        g_last_shot_t = 0.0
        track_ensure_for_engage()
        g_ir_done = False
        fire_ir_warn_once()
        g_fire_phase = FIRE_PHASE_SHOOT
        g_phase_t0 = now_s()
        fire_bead_burst_start()
        log(
            "FIRE start aim+shoot on=%.1fs off=%.1fs track=(%.2f,%.2f) ir=%s shots0=%d"
            % (T_FIRE_ON, T_FIRE_OFF, g_track_x, g_track_y, str(g_ir_done), g_burst_shots)
        )


def tick_patrol():
    """
    蓝线巡线 T_MOVE 秒后进 SCAN。
    有线：按官方 PID 贴线；无线：停车；持续 LINE_LOST_S 秒无线才 SCAN。
    """
    global g_patrol_line_t0, g_line_log_t
    line_update()
    if g_line_hit < 1:
        # 本帧无线：停车（官方循环只是不发指令；我们停车防冲出）
        try:
            gimbal_ctrl.rotate_with_speed(0, 0)
        except Exception:
            pass
        chassis_halt()
        if g_line_ever_ok and line_stable_false():
            line_follow_log_snapshot("LOST")
            log("PATROL leave no_line lost_s>=%.1f miss=%d -> SCAN" % (
                LINE_LOST_S, g_line_miss
            ))
            g_patrol_line_t0 = 0.0
            g_line_log_t = 0.0
            person_hit_reset()
            set_state(STATE_SCAN, "no_line_stable")
            return
        if g_line_ever_ok == False and state_age() >= 4.0:
            line_follow_log_snapshot("never")
            log("PATROL leave no_line_timeout age=%.1fs -> SCAN" % state_age())
            g_patrol_line_t0 = 0.0
            g_line_log_t = 0.0
            set_state(STATE_SCAN, "no_line_timeout")
            return
        return
    # 有线：官方贴线（先 step 再 log，避免 start 时 err/yaw 仍为 0）
    if g_patrol_line_t0 <= 0.0:
        g_patrol_line_t0 = now_s()
        g_line_log_t = 0.0
        line_follow_step()
        line_follow_log_snapshot("start")
    else:
        line_follow_step()
        line_follow_log_tick()
    if (now_s() - g_patrol_line_t0) >= T_MOVE:
        line_follow_log_snapshot("done")
        g_patrol_line_t0 = 0.0
        g_line_log_t = 0.0
        person_hit_reset()
        set_state(STATE_SCAN, "follow_time_up")

def scan_look_should_keep():
    """本角位是否继续查人：look_ops < SCAN_LOOK_OPS 则继续，满 5 则应转角。"""
    lo = SCAN_LOOK_OPS
    if lo < 1:
        lo = 1
    return g_scan_look_ops < lo

def tick_scan_common():
    """
    SCAN（behavior-spec）：
      每角最多 SCAN_LOOK_OPS 次查人；
      仅 hit≥PERSON_HIT_NEED 才算发现 → FIRE；
      hit 1～2 不算发现，满 5 次转下一角。
    """
    global g_scan_look_ops
    chassis_halt()

    # ----- LOOK：未满 5 次则采样 -----
    if scan_look_should_keep():
        found, frame_hit, dt_look = scan_look_once()
        # 唯一进 FIRE 条件：连续 hit 已达门槛
        if found:
            # 底盘保持停；不回中。记忆框优先本帧，否则用扫描中最后一次有效检出
            chassis_halt()
            engage_fire_immediate("person_on_scan")
            return
        if scan_look_should_keep():
            return
        # 刚满 5 次且未 found → 下面 TURN

    # ----- TURN：本角 5 次内未 hit≥3，放弃本角（含仅 hit1～2）-----
    if g_person_hit > 0 and g_person_hit < PERSON_HIT_NEED:
        log(
            "SCAN no-found hit=%d<%d look_ops=%d -> next angle"
            % (g_person_hit, PERSON_HIT_NEED, g_scan_look_ops)
        )
    person_hit_reset()
    seg_done, did_turn = scan_tick_turn()
    if did_turn == False and seg_done == False:
        return
    if seg_done == False:
        return

    log("SCAN seg done %s plan_step=%d" % (g_scan_seg_name, g_scan_step_i))
    adv = scan_advance_or_finish()
    if adv == "next":
        person_hit_reset()
        return

    # 完整 SCAN 循环结束且无人：看线决定 PATROL 或原地再扫
    scan_log_cycle_summary("empty_done")
    gimbal_fast_home("scan_queue_done", keep_scan_pitch=False)
    i = 0
    while i < LINE_CONFIRM_FRAMES:
        line_update()
        i = i + 1
    if line_stable_true():
        log("SCAN full cycle empty + has line -> PATROL")
        set_state(STATE_PATROL, "scan_empty_has_line")
        return
    # 规格：找不到线 → 原地反复 SCAN（无空扫次数上限）
    log("SCAN full cycle empty + no line -> SCAN again")
    set_state(STATE_SCAN, "scan_empty_no_line_repeat")

def tick_scan():
    tick_scan_common()

def tick_fire():
    """
    底盘停；跟记忆框瞄准；SHOOT 3s / HOLD 3s。
    丢人不停射、不回中；仅首段射击完成后才允许 miss 放弃。
    """
    global g_fire_phase, g_phase_t0, g_min_fire_done
    chassis_halt()
    person_track_update()
    aim_pid_track()

    if person_confirmed_lost():
        log("FIRE give_up miss=%d min_fire_done=1 shots=%d -> SCAN" % (
            g_person_miss, g_burst_shots
        ))
        leave_combat_to_rescan("fire_give_up")
        return

    if g_fire_phase == FIRE_PHASE_SHOOT:
        if phase_age() < T_FIRE_ON:
            fire_bead_burst_tick()
            return
        fire_bead_burst_stop()
        g_min_fire_done = True
        fx_fire_wait_led()
        g_fire_phase = FIRE_PHASE_HOLD
        g_phase_t0 = now_s()
        log("FIRE shoot done shots=%d hold %.1fs" % (g_burst_shots, T_FIRE_OFF))
        return

    if g_fire_phase == FIRE_PHASE_HOLD:
        if phase_age() >= T_FIRE_OFF:
            g_fire_phase = FIRE_PHASE_SHOOT
            g_phase_t0 = now_s()
            fire_bead_burst_start()
            log("FIRE shoot again %.1fs" % T_FIRE_ON)
        return


# =============================================================================
# ENTRY — 启动与主循环
# =============================================================================
def setup():
    log("setup begin")
    mode_ensure_free("setup")
    set_gimbal_speed(GIMBAL_YAW_SPEED)
    gimbal_pose_line()
    vision_ctrl.enable_detection(rm_define.vision_detection_people)
    vision_ctrl.enable_detection(rm_define.vision_detection_line)
    vision_ctrl.line_follow_color_set(rm_define.line_follow_color_blue)
    media_ctrl.exposure_value_update(rm_define.exposure_value_small)
    gun_ctrl.set_fire_count(FIRE_BEADS_PER_PULSE)
    try:
        ir_blaster_ctrl.set_fire_count(1)
    except Exception:
        pass
    fx_patrol()
    pid_reset_aim()
    person_hit_reset()
    log("setup done v1.38.1 hit_need=%d miss_need=%d fire_on=%.1fs" % (
        PERSON_HIT_NEED, PERSON_MISS_NEED, T_FIRE_ON
    ))

def start():
    global g_state
    print("======== Line Guard start ========")
    print("# LINE_GUARD_VERSION=1.38.1 stamp=2026-08-07 13:45:00")
    log("program start")
    setup()
    set_state(STATE_PATROL, "boot")
    while True:
        if g_state == STATE_PATROL:
            tick_patrol()
        elif g_state == STATE_SCAN:
            tick_scan()
        elif g_state == STATE_FIRE:
            tick_fire()
        else:
            log("bad state")
            set_state(STATE_PATROL, "bad_state")
