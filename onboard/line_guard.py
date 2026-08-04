# LINE_GUARD_VERSION=1.17.0 stamp=2026-08-04 17:10:00  (paste this whole file; check stamp matches latest)
# -*- coding: utf-8 -*-
# S1 Line Guard v1.17 — 单文件粘贴进 App 实验室
#
# =============================================================================
# 代码纪律（S1 单文件）
# =============================================================================
# - 全部逻辑仅本文件；按分块改，避免动到已稳定模块
# - SCAN / LOCK / FIRE 已调稳：默认冻结，除非联调明确要求
# - 巡线相关：LINE_*、LINE VISION、LINE FOLLOW、tick_patrol、模式切换
#
# =============================================================================
# 巡线（v1.14 = 官方示例一字不差的数据路径）
# =============================================================================
# 官方能识别你瓷砖蓝线的程序要点：
#   robot_mode_chassis_follow
#   gimbal.rotate_with_degree(gimbal_down, 20)   # 相对低头，不是 pitch_ctrl 绝对角
#   enable_detection(line) + line_follow_color_blue
#   list_LineList = RmList(get_line_detection_info())   # 必须 RmList
#   if len==42 and list_LineList[2] >= 1: x = list_LineList[19]
#   pid.set_error(x-0.5); gimbal.rotate_with_speed(pid.get_output(),0)
#   chassis.set_trans_speed(0.2); chassis.move(0)
# 我们之前用裸 list 下标，在 S1 上与 RmList 语义不一致 → 一直 pts=0。
# SCAN/LOCK/FIRE 仍 robot_mode_free。
#
# v1.16（日志 16:47）：线上仍秒锁人开火
#   原因：tick_patrol 先查人再查线；操作者在画面 → person hit=3 抢在 T_MOVE 前
#   修复：PATROL 阶段不认人（只循线 T_MOVE 再 SCAN）；SCAN 才锁人
#   射击：fire_continuous 非阻塞 + 主循环计时 stop（不连 fire_once）
# =============================================================================

# =============================================================================
# CONFIG
# =============================================================================
T_MOVE = 6.0
T_CLEAR = 1.2
PERSON_MISS_NEED = 12
PERSON_HIT_NEED = 3
# SCAN 中认人更严
PERSON_HIT_NEED_SCAN = 8
# PATROL 循线时不锁人（必须先走完 T_MOVE 再扫人）
PERSON_LOCK_ON_PATROL = False
LOOP_DT = 0.05
LOG_HEARTBEAT_S = 1.0
T_PERSON_COOLDOWN = 12.0
T_FIRE_MAX = 8.0
FIRE_MAX_ROUNDS = 1
SCAN_EMPTY_MAX = 1
# 开火前必须当前帧仍看见人，且框足够大（减误 IR）
PERSON_FIRE_MIN_W = 0.10
PERSON_FIRE_MIN_H = 0.12

# 俯仰：巡线低头 / 扫描上扬（文档 pitch 范围约 -20~+35）
PITCH_LINE = -20
PITCH_SCAN = 20

# --- SCAN ---
SCAN_HALF = 180.0
SCAN_SIDE_A = SCAN_HALF
SCAN_SIDE_B = -SCAN_HALF
SCAN_YAW_SPEED = 120.0
YAW_ARRIVE = 8.0
SCAN_STUCK_FRAMES = 12
HOME_YAW_SPEED = 500.0
HOME_TIMEOUT_S = 3.0

# --- PATROL 巡线（与官方 PID 示例一致）---
LINE_SPEED = 0.30
# 官方 pid.set_ctrl_params(330, 0, 28)
LINE_PID_KP = 330.0
LINE_PID_KI = 0.0
LINE_PID_KD = 28.0
LINE_GIMBAL_YAW_MAX = 300.0
LINE_CX_DEADZONE = 0.02
LINE_CONFIRM_FRAMES = 3
LINE_LOST_FRAMES = 12
# 进 PATROL 时相对低头度数（官方 rotate_with_degree(down, 20)）
LINE_LOOK_DOWN_DEG = 20

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
# 水弹：连击 = 短间隔 fire_once（continuous 官方约 1 发/s，体感不像连击）
# 主循环仍可跑；单次 fire_once 会短暂阻塞，间隔 0.22s 折中
T_BURST_ON = 2.0
T_BURST_WAIT = 3.0
FIRE_PULSE_INTERVAL = 0.22
FIRE_BEADS_PER_PULSE = 2
# True=脉冲连击；False=仅 fire_continuous（约 1Hz）
FIRE_USE_PULSE = True

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
g_line_info_len = 0
g_line_pts = 0
g_line_ever_ok = False
g_line_pid = None
g_line_look_done = False
g_person_ignore_until = 0.0
g_scan_empty_count = 0
g_fire_rounds = 0

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
# 当前机器人模式标签（仅日志；实际以 set_mode 为准）
g_mode_tag = "free"

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

def get_pitch():
    return gimbal_ctrl.get_axis_angle(rm_define.gimbal_axis_pitch)

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

def person_cooldown_active():
    return now_s() < g_person_ignore_until

def person_cooldown_start(sec, reason):
    global g_person_ignore_until
    g_person_ignore_until = now_s() + sec
    person_hit_reset()
    log("PERSON cooldown %.1fs | %s" % (sec, reason))

def people_seen():
    if person_cooldown_active():
        return False
    ok, x, y, w, h = people_get_first()
    return ok

def person_hit_reset():
    global g_person_hit
    g_person_hit = 0

def person_hit_update(need):
    """
    进 LOCK 前防抖。need 为所需连续帧数。
    冷却期内永不确认。
    """
    global g_person_hit
    if person_cooldown_active():
        g_person_hit = 0
        return False
    if people_seen():
        g_person_hit = g_person_hit + 1
    else:
        g_person_hit = 0
    return g_person_hit >= need

def person_track_update():
    """
    LOCK/FIRE 跟瞄用。冷却期视为一直 miss（尽快退出战斗态）。
    """
    global g_person_miss, g_no_person_t0
    global g_last_px, g_last_py, g_have_last_person
    if person_cooldown_active():
        g_person_miss = g_person_miss + 1
        return False
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

def leave_combat_to_recover(reason):
    """射击/锁定结束后：停火、行人冷却、去找线（打断 SCAN/FIRE 死循环）。"""
    fire_stop()
    person_cooldown_start(T_PERSON_COOLDOWN, reason)
    set_state(STATE_RECOVER, reason)

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

# =============================================================================
# LINE VISION — 必须 RmList，与官方程序相同下标
# =============================================================================
def line_get_rmlist():
    """官方：list_LineList = RmList(vision_ctrl.get_line_detection_info())"""
    raw = vision_ctrl.get_line_detection_info()
    try:
        return RmList(raw)
    except Exception:
        return raw

def line_read():
    """
    官方判定（一字不差语义）：
      if len(list_LineList) == 42:
          if list_LineList[2] >= 1:
              variable_x = list_LineList[19]
    返回 (ok, cx, n, pts)
    """
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
    # 官方严格 len==42
    if n == 42 and pts >= 1:
        try:
            cx = float(info[19])
            return True, cx, n, pts
        except Exception:
            return False, 0.5, n, pts
    # 少数固件长度略异，仅当点数有效时放宽
    if n >= 42 and pts >= 1:
        try:
            cx = float(info[19])
            return True, cx, n, pts
        except Exception:
            return False, 0.5, n, pts
    return False, 0.5, n, pts

def line_info_raw():
    """兼容旧调用名。"""
    return line_get_rmlist()

def line_raw_seen():
    if FORCE_NO_LINE:
        return False
    ok, cx, n, pts = line_read()
    return ok

def line_update():
    global g_line_hit, g_line_miss, g_line_cx, g_line_info_len, g_line_pts, g_line_ever_ok
    if FORCE_NO_LINE:
        g_line_miss = g_line_miss + 1
        g_line_hit = 0
        g_line_pts = 0
        return
    ok, cx, n, pts = line_read()
    g_line_info_len = n
    g_line_pts = pts
    if ok:
        g_line_cx = cx
        g_line_hit = g_line_hit + 1
        g_line_miss = 0
        g_line_ever_ok = True
    else:
        g_line_miss = g_line_miss + 1
        g_line_hit = 0

def line_stable_true():
    return g_line_hit >= LINE_CONFIRM_FRAMES

def line_stable_false():
    need = LINE_CONFIRM_FRAMES
    if g_patrol_line_t0 > 0.0:
        need = LINE_LOST_FRAMES
    return g_line_miss >= need

def line_pid_init():
    """官方 pid_line = PIDCtrl(); set_ctrl_params(330,0,28)"""
    global g_line_pid
    g_line_pid = None
    try:
        g_line_pid = PIDCtrl()
        g_line_pid.set_ctrl_params(LINE_PID_KP, LINE_PID_KI, LINE_PID_KD)
        log("LINE PIDCtrl ok params=%.0f,%.0f,%.0f" % (LINE_PID_KP, LINE_PID_KI, LINE_PID_KD))
        return
    except Exception:
        pass
    try:
        g_line_pid = rm_ctrl.PIDCtrl()
        g_line_pid.set_ctrl_params(LINE_PID_KP, LINE_PID_KI, LINE_PID_KD)
        log("LINE rm_ctrl.PIDCtrl ok")
        return
    except Exception:
        g_line_pid = None
        log("LINE PIDCtrl unavailable, use P gain only")

def line_look_down_official():
    """
    官方：gimbal_ctrl.rotate_with_degree(rm_define.gimbal_down, 20)
    相对低头；先尽量归中再低头，避免叠加速度。
    """
    global g_line_look_done
    try:
        gimbal_ctrl.rotate_with_speed(0, 0)
    except Exception:
        pass
    try:
        gimbal_ctrl.yaw_ctrl(0)
    except Exception:
        pass
    time.sleep(0.15)
    try:
        # 先到水平附近再相对下压（与官方从默认姿态 down 20 类似）
        gimbal_ctrl.pitch_ctrl(0)
        time.sleep(0.2)
    except Exception:
        pass
    try:
        gimbal_ctrl.rotate_with_degree(rm_define.gimbal_down, LINE_LOOK_DOWN_DEG)
        g_line_look_done = True
        log("LINE look_down relative %d deg" % LINE_LOOK_DOWN_DEG)
    except Exception:
        # 兜底绝对角
        try:
            gimbal_ctrl.pitch_ctrl(-LINE_LOOK_DOWN_DEG)
            g_line_look_done = True
            log("LINE look_down fallback pitch=%d" % (-LINE_LOOK_DOWN_DEG))
        except Exception:
            log("LINE look_down FAIL")
    time.sleep(0.25)

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
    try:
        pit = get_pitch()
    except Exception:
        pit = 0.0
    if g_state == STATE_PATROL:
        age = 0.0
        if g_patrol_line_t0 > 0:
            age = t - g_patrol_line_t0
        extra = " mode=%s lineHit=%d miss=%d follow=%.1f pitch=%.0f cx=%.2f err=%.2f gyaw=%.0f n=%d pts=%d" % (
            g_mode_tag, g_line_hit, g_line_miss, age, pit,
            g_line_cx, g_line_err, g_line_yaw_spd, g_line_info_len, g_line_pts
        )
    elif g_state == STATE_SCAN or g_state == STATE_LOST_SCAN:
        extra = " mode=%s qi=%d/%d seg=%s tgt=%.0f yaw=%.0f pitch=%.0f pHit=%d person=%s" % (
            g_mode_tag, g_scan_qi, len(g_scan_queue), g_scan_seg_name, g_scan_target_yaw,
            get_yaw(), pit, g_person_hit, str(has_p)
        )
    elif g_state == STATE_LOCK or g_state == STATE_FIRE:
        extra = " mode=%s person=%s miss=%d xy=(%.2f,%.2f) fphase=%d ir=%s shots=%d" % (
            g_mode_tag, str(has_p), g_person_miss, px, py, g_fire_phase, str(g_ir_done), g_burst_shots
        )
    else:
        extra = " mode=%s lineHit=%d person=%s pitch=%.0f" % (
            g_mode_tag, g_line_hit, str(has_p), pit
        )
    log("HB" + extra)

# =============================================================================
# ACTUATORS
# =============================================================================
def gimbal_set_pitch_line():
    """低头看线：绝对 pitch=PITCH_LINE（约 -20）。"""
    set_gimbal_speed(HOME_YAW_SPEED)
    try:
        gimbal_ctrl.pitch_ctrl(PITCH_LINE)
    except Exception:
        pass

def gimbal_set_pitch_scan():
    """扫描/跟瞄上扬：绝对 pitch=PITCH_SCAN（约 +20）。"""
    set_gimbal_speed(HOME_YAW_SPEED)
    try:
        gimbal_ctrl.pitch_ctrl(PITCH_SCAN)
    except Exception:
        pass

def gimbal_pose_line():
    """回巡线姿态：yaw=0 且 pitch=低头。用 angle_ctrl 一次到位更可靠。"""
    set_gimbal_speed(HOME_YAW_SPEED)
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
    time.sleep(0.12)
    # 再确认低头（部分机上 yaw 指令后 pitch 会漂）
    try:
        gimbal_ctrl.pitch_ctrl(PITCH_LINE)
    except Exception:
        pass
    log("POSE line yaw=%.0f pitch=%.0f" % (get_yaw(), get_pitch()))

def gimbal_pose_scan_yaw0():
    """扫描起点：yaw=0 且 pitch=上扬。"""
    set_gimbal_speed(HOME_YAW_SPEED)
    try:
        gimbal_ctrl.angle_ctrl(0, PITCH_SCAN)
    except Exception:
        try:
            gimbal_ctrl.yaw_ctrl(0)
        except Exception:
            pass
        try:
            gimbal_ctrl.pitch_ctrl(PITCH_SCAN)
        except Exception:
            pass
    time.sleep(0.1)
    try:
        gimbal_ctrl.pitch_ctrl(PITCH_SCAN)
    except Exception:
        pass
    log("POSE scan yaw=%.0f pitch=%.0f" % (get_yaw(), get_pitch()))

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

# =============================================================================
# ROBOT MODE（云台跟随 vs 自由云台）— 所有状态切换必须走这里
# =============================================================================
def mode_ensure_free(reason):
    """
    进 SCAN / LOST_SCAN / LOCK / FIRE / RECOVER 前调用。
    停底盘 + 停云台速度 + robot_mode_free，避免「云台跟随」带着底盘乱动。
    """
    global g_mode_tag
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
    g_mode_tag = "free"
    log("MODE free | %s" % reason)

def mode_ensure_line_follow(reason):
    """官方：robot_ctrl.set_mode(rm_define.robot_mode_chassis_follow)"""
    global g_mode_tag
    robot_ctrl.set_mode(rm_define.robot_mode_chassis_follow)
    g_mode_tag = "chassis_follow"
    log("MODE chassis_follow | %s" % reason)

def mode_free_halt():
    mode_ensure_free("mode_free_halt")

def mode_gimbal_lead():
    mode_ensure_line_follow("compat")

def mode_ensure_gimbal_lead(reason):
    mode_ensure_line_follow(reason)

def line_follow_step():
    """
    官方 while 循环体：
      list_LineList=RmList(get_line_detection_info())
      if len==42 and [2]>=1:
          x=[19]; pid.set_error(x-0.5)
          gimbal.rotate_with_speed(pid.get_output(), 0)
          chassis.set_trans_speed(0.2); chassis.move(0)
      else:
          gimbal.rotate_with_speed(0,0)
    """
    global g_line_cx, g_line_err, g_line_yaw_spd, g_line_info_len, g_line_pts
    ok, cx, n, pts = line_read()
    g_line_info_len = n
    g_line_pts = pts
    if ok == False:
        g_line_yaw_spd = 0.0
        g_line_err = 0.0
        try:
            gimbal_ctrl.rotate_with_speed(0, 0)
        except Exception:
            pass
        chassis_halt()
        return
    g_line_cx = cx
    err = cx - 0.5
    g_line_err = err
    yaw_spd = 0.0
    if abs(err) >= LINE_CX_DEADZONE:
        if g_line_pid is not None:
            try:
                g_line_pid.set_error(err)
                yaw_spd = g_line_pid.get_output()
            except Exception:
                yaw_spd = err * LINE_PID_KP
        else:
            yaw_spd = err * LINE_PID_KP
        yaw_spd = clamp(yaw_spd, -LINE_GIMBAL_YAW_MAX, LINE_GIMBAL_YAW_MAX)
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

def person_fire_ok():
    """
    允许 IR/水弹：当前帧真有人，且框够大（过滤远处噪点/地面误检）。
    """
    if person_cooldown_active():
        return False
    ok, x, y, w, h = people_get_first()
    if ok == False:
        return False
    try:
        if w < PERSON_FIRE_MIN_W or h < PERSON_FIRE_MIN_H:
            return False
    except Exception:
        return False
    return True

def fire_ir_warn_once():
    """红外示警一次；无人/框太小则跳过（避免空放 IR）。"""
    global g_fire_count, g_ir_done
    if g_ir_done:
        log("IR_WARN skip already")
        return
    if person_fire_ok() == False:
        log("IR_WARN skip no solid person")
        # 不置 ir_done，若之后框变实可再试；但 LOCK 超时会丢
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
        gun_ctrl.set_fire_count(1)
        gun_ctrl.fire_once()
        log("IR_WARN fallback gun fire_once")
    g_fire_count = g_fire_count + 1
    g_ir_done = True

def fire_bead_burst_start():
    """
    连击窗口开始。
    FIRE_USE_PULSE=True：短间隔 fire_once（连击手感，单次略阻塞）
    False：fire_continuous（非阻塞，官方约 1 发/s）
    """
    global g_last_shot_t, g_burst_shots, g_fire_count
    if ENABLE_FIRE == False:
        log("BURST_ON skip ENABLE_FIRE=0")
        return
    if person_fire_ok() == False:
        log("BURST_ON skip no solid person")
        return
    fx_fire_burst_led()
    g_last_shot_t = 0.0
    g_burst_shots = 0
    if FIRE_USE_PULSE == False:
        try:
            gun_ctrl.set_fire_count(FIRE_BEADS_PER_PULSE)
            gun_ctrl.fire_continuous()
            g_last_shot_t = now_s()
            g_burst_shots = 1
            log("BURST_ON continuous %.1fs" % T_BURST_ON)
        except Exception:
            log("BURST continuous FAIL")
        return
    # 脉冲连击：立刻第一发
    fire_bead_pulse_once()
    log("BURST_ON pulse interval=%.2fs beads=%d for %.1fs" % (
        FIRE_PULSE_INTERVAL, FIRE_BEADS_PER_PULSE, T_BURST_ON
    ))

def fire_bead_pulse_once():
    global g_last_shot_t, g_burst_shots, g_fire_count
    if ENABLE_FIRE == False:
        return
    if person_fire_ok() == False:
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
        g_fire_count = g_fire_count + 1
        g_last_shot_t = now_s()
        log("BURST pulse #%d beads=%d" % (g_burst_shots, n))
    except Exception:
        log("BURST pulse fail")

def fire_bead_burst_tick():
    """连击窗口内：脉冲补发或 continuous 保活；无人则停。"""
    if ENABLE_FIRE == False:
        return
    if person_fire_ok() == False:
        try:
            gun_ctrl.stop()
        except Exception:
            pass
        return
    if FIRE_USE_PULSE == False:
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
    log("BURST_STOP shots=%d" % g_burst_shots)

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
    """
    回 yaw=0。
    keep_scan_pitch=True  → 保持/设为扫描俯仰（扫前归零）
    keep_scan_pitch=False → 同时低头到 PITCH_LINE（扫完回巡线）
    """
    gimbal_stop()
    fx_recenter()
    yaw0 = get_yaw()
    pit0 = 0.0
    try:
        pit0 = get_pitch()
    except Exception:
        pit0 = 0.0
    log("HOME begin yaw=%.0f pitch=%.0f | %s" % (yaw0, pit0, reason))
    set_gimbal_speed(HOME_YAW_SPEED)
    if keep_scan_pitch:
        # 扫人姿态：0 航向 + 上扬
        try:
            gimbal_ctrl.angle_ctrl(0, PITCH_SCAN)
        except Exception:
            try:
                gimbal_ctrl.yaw_ctrl(0)
            except Exception:
                pass
            gimbal_set_pitch_scan()
        time.sleep(0.1)
        gimbal_set_pitch_scan()
    else:
        # 巡线姿态：0 航向 + 低头（angle_ctrl 一次锁死两轴）
        gimbal_pose_line()
    gimbal_stop()
    log("HOME done yaw=%.0f pitch=%.0f" % (get_yaw(), get_pitch()))

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
    # SCAN 全程 free，禁止云台跟随带着底盘转
    mode_ensure_free("scan_start_full")
    g_scan_queue = scan_queue_full_two_rounds()
    person_hit_reset()
    # 扫前：必须上扬 + 尽量在中心起扫
    if abs(get_yaw()) > YAW_ARRIVE:
        gimbal_fast_home("scan_start_ensure_center", keep_scan_pitch=True)
        mode_ensure_free("scan_after_home")
    else:
        gimbal_pose_scan_yaw0()
    if len(g_scan_queue) <= 0:
        log("SCAN empty queue")
        return
    fx_scan()
    # 再钉一次扫描俯仰，防止 home 后漂
    gimbal_set_pitch_scan()
    scan_load_segment(0)
    log("SCAN full 2-round start n=%d A=%+.0f B=%+.0f spd=%.0f pitch=%d | %s" % (
        len(g_scan_queue), SCAN_SIDE_A, SCAN_SIDE_B, SCAN_YAW_SPEED, PITCH_SCAN, reason
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
    """
    状态入口。模式纪律（强制）：
      - 目标态不是 PATROL → 一律 mode_ensure_free
      - 目标态是 PATROL → free 摆姿后 mode_ensure_gimbal_lead
    """
    global g_state, g_state_t0, g_no_person_t0, g_person_miss
    global g_patrol_line_t0, g_fire_count, g_fire_phase, g_phase_t0
    global g_ir_done, g_line_hit, g_line_miss, g_have_last_person
    global g_last_px, g_last_py, g_line_ever_ok, g_fire_rounds
    old = g_state
    g_state = s
    g_state_t0 = now_s()
    if s != STATE_FIRE:
        g_no_person_t0 = now_s()
        g_person_miss = 0
    log("STATE %s -> %s | %s" % (state_name(old), state_name(s), reason))

    # ----- 非 PATROL：必须先切 free，再做该态动作 -----
    if s != STATE_PATROL:
        fire_stop()
        mode_ensure_free("enter_%s" % state_name(s))

    # ----- PATROL：官方 chassis_follow + 相对低头 + RmList 读线 -----
    if s == STATE_PATROL:
        fire_stop()
        mode_ensure_free("patrol_before_pose")
        # 先 free 下归中，再切官方循线模式
        try:
            gimbal_ctrl.yaw_ctrl(0)
            gimbal_ctrl.pitch_ctrl(0)
        except Exception:
            pass
        time.sleep(0.15)
        mode_ensure_line_follow("patrol_start")
        line_pid_init()
        line_look_down_official()
        fx_patrol()
        g_patrol_line_t0 = 0.0
        g_line_hit = 0
        g_line_miss = 0
        g_line_cx = 0.5
        g_line_err = 0.0
        g_line_yaw_spd = 0.0
        g_line_ever_ok = False
        person_hit_reset()
        vision_ctrl.enable_detection(rm_define.vision_detection_line)
        vision_ctrl.line_follow_color_set(rm_define.line_follow_color_blue)
        log("PATROL ready official line spd=%.2f pid=%.0f" % (LINE_SPEED, LINE_PID_KP))

    # ----- SCAN：free 下完整两遍 -----
    if s == STATE_SCAN:
        # free 已在上方 ensure
        scan_start_full("normal_scan")

    # ----- LOST_SCAN：free 快回中 + 完整两遍 -----
    if s == STATE_LOST_SCAN:
        pid_reset_aim()
        g_have_last_person = False
        fx_person_lost()
        time.sleep(0.2)
        # home 期间保持 free
        mode_ensure_free("lost_before_home")
        gimbal_fast_home("lost_before_rescan", keep_scan_pitch=True)
        mode_ensure_free("lost_before_scan")
        scan_start_full("lost_rescan_full")

    # ----- LOCK：free 下跟瞄 -----
    if s == STATE_LOCK:
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

    # ----- FIRE -----
    if s == STATE_FIRE:
        mode_ensure_free("enter_FIRE")
        pid_reset_aim()
        g_fire_rounds = 0
        if g_ir_done:
            g_fire_phase = FIRE_PHASE_IR_DONE
        else:
            g_fire_phase = FIRE_PHASE_AIM
            fx_lock()
        g_phase_t0 = now_s()
        g_person_miss = 0
        g_no_person_t0 = now_s()
        log("FIRE enter ir_done=%s max_s=%.1f rounds<=%d" % (
            str(g_ir_done), T_FIRE_MAX, FIRE_MAX_ROUNDS
        ))

    # ----- RECOVER：找线（冷却期内不锁人）-----
    if s == STATE_RECOVER:
        pid_reset_aim()
        gimbal_pose_line()
        fx_recover()
        g_line_hit = 0
        g_line_miss = 0
        person_hit_reset()
        log("RECOVER find line pitch=%.0f cd=%s" % (
            get_pitch(), str(person_cooldown_active())
        ))

def tick_patrol():
    """
    PATROL = 官方循线 T_MOVE 秒，再 SCAN。
    默认 PERSON_LOCK_ON_PATROL=False：巡线阶段不认人，避免操作者秒锁开火。
    """
    global g_patrol_line_t0, g_scan_empty_count
    line_update()
    # 先保证线：只在明确允许时才锁人
    if PERSON_LOCK_ON_PATROL:
        if person_hit_update(PERSON_HIT_NEED):
            log("PATROL person hit=%d -> LOCK" % g_person_hit)
            set_state(STATE_LOCK, "person_on_patrol")
            return
    if line_stable_false():
        if g_line_ever_ok:
            g_patrol_line_t0 = 0.0
            person_hit_reset()
            set_state(STATE_SCAN, "no_line_stable")
            return
        if state_age() >= 4.0:
            log("PATROL no line ever n=%d pts=%d -> SCAN" % (g_line_info_len, g_line_pts))
            g_patrol_line_t0 = 0.0
            set_state(STATE_SCAN, "no_line_timeout")
            return
        try:
            gimbal_ctrl.rotate_with_speed(0, 0)
        except Exception:
            pass
        chassis_halt()
        return
    if line_stable_true() == False:
        try:
            gimbal_ctrl.rotate_with_speed(0, 0)
        except Exception:
            pass
        chassis_halt()
        return
    # 成功贴线：清空「空扫」计数
    g_scan_empty_count = 0
    if g_patrol_line_t0 <= 0.0:
        g_patrol_line_t0 = now_s()
        mode_ensure_line_follow("patrol_follow_begin")
        log("PATROL follow start cx=%.2f pts=%d n=%d pitch=%.0f" % (
            g_line_cx, g_line_pts, g_line_info_len, get_pitch()
        ))
    line_follow_step()
    if (now_s() - g_patrol_line_t0) >= T_MOVE:
        age = now_s() - g_patrol_line_t0
        g_patrol_line_t0 = 0.0
        person_hit_reset()
        log("PATROL follow done age=%.2fs need=%.1f -> SCAN" % (age, T_MOVE))
        set_state(STATE_SCAN, "follow_time_up")

def tick_scan_common(is_lost):
    """
    SCAN / LOST_SCAN：
      冷却期不锁人；认人更严；扫完无蓝线不得无限重扫。
    """
    global g_scan_empty_count
    chassis_halt()
    if person_hit_update(PERSON_HIT_NEED_SCAN):
        gimbal_stop()
        log("SCAN person hit=%d need=%d -> LOCK" % (g_person_hit, PERSON_HIT_NEED_SCAN))
        set_state(STATE_LOCK, "person_on_scan")
        return
    if scan_tick_turn() == False:
        return
    person_hit_reset()
    log("SCAN seg done %s yaw=%.0f" % (g_scan_seg_name, get_yaw()))
    adv = scan_advance_or_finish()
    if adv == "next":
        return
    gimbal_fast_home("scan_two_rounds_done", keep_scan_pitch=False)
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
        g_scan_empty_count = 0
        log("SCAN done line -> PATROL")
        set_state(STATE_PATROL, "scan_has_line")
        return
    # 无蓝线：限制重扫次数，避免 SCAN 死循环
    g_scan_empty_count = g_scan_empty_count + 1
    if g_scan_empty_count > SCAN_EMPTY_MAX:
        log("SCAN empty x%d -> RECOVER (break loop)" % g_scan_empty_count)
        g_scan_empty_count = 0
        person_cooldown_start(T_PERSON_COOLDOWN, "scan_empty_break")
        set_state(STATE_RECOVER, "scan_empty_break")
        return
    log("SCAN done no line -> SCAN again (%d/%d)" % (g_scan_empty_count, SCAN_EMPTY_MAX))
    set_state(STATE_SCAN, "rescan_no_line")

def tick_scan():
    tick_scan_common(False)

def tick_lost_scan():
    tick_scan_common(True)

def tick_lock():
    global g_fire_phase, g_phase_t0
    chassis_halt()
    person_track_update()
    aim_pid_track(LOOP_DT)
    if person_confirmed_lost():
        log("LOCK lost -> RECOVER (no LOST_SCAN loop)")
        leave_combat_to_recover("lock_lost")
        return
    # 冷却开始后也可能从 LOCK 进来：直接撤
    if person_cooldown_active():
        leave_combat_to_recover("lock_cooldown")
        return
    if g_ir_done == False and state_age() >= T_AIM_BEFORE_IR:
        if person_fire_ok() == False:
            # 超时仍无实框：不当入侵，回找线
            if state_age() >= (T_AIM_BEFORE_IR + 1.5):
                log("LOCK aim timeout no solid person -> RECOVER")
                leave_combat_to_recover("lock_no_solid_person")
            return
        fire_ir_warn_once()
        if g_ir_done:
            g_fire_phase = FIRE_PHASE_IR_DONE
            g_phase_t0 = now_s()
            log("LOCK IR warn -> FIRE")
            set_state(STATE_FIRE, "after_ir_warn")

def tick_fire():
    """
    FIRE：有限轮连射 + 总时长上限，结束后 RECOVER（打断死循环）。
    fire_once 会阻塞，轮数/时长限制也减轻「停不掉」体感。
    """
    global g_fire_phase, g_phase_t0, g_fire_rounds
    chassis_halt()
    person_track_update()

    # 总时长到：强制退出战斗
    if state_age() >= T_FIRE_MAX:
        log("FIRE max time %.1fs -> RECOVER" % T_FIRE_MAX)
        leave_combat_to_recover("fire_max_time")
        return

    if person_confirmed_lost():
        log("FIRE lost -> RECOVER")
        leave_combat_to_recover("fire_lost")
        return

    aim_pid_track(LOOP_DT)

    if g_ir_done == False:
        if state_age() >= T_AIM_BEFORE_IR:
            if person_fire_ok() == False:
                if state_age() >= T_FIRE_MAX * 0.5:
                    leave_combat_to_recover("fire_no_solid_person")
                return
            fire_ir_warn_once()
            if g_ir_done == False:
                return
            g_fire_phase = FIRE_PHASE_BURST_ON
            g_phase_t0 = now_s()
            g_fire_rounds = 1
            fire_bead_burst_start()
        return

    if g_fire_phase == FIRE_PHASE_IR_DONE:
        if person_fire_ok() == False:
            leave_combat_to_recover("fire_ir_done_no_person")
            return
        g_fire_phase = FIRE_PHASE_BURST_ON
        g_phase_t0 = now_s()
        g_fire_rounds = 1
        fire_bead_burst_start()
        return

    if g_fire_phase == FIRE_PHASE_BURST_ON:
        if phase_age() < T_BURST_ON:
            fire_bead_burst_tick()
            return
        fire_bead_burst_stop()
        # 默认只打一轮就撤，避免操作者站在旁边无限 2s/3s
        if g_fire_rounds >= FIRE_MAX_ROUNDS:
            log("FIRE rounds=%d done -> RECOVER" % g_fire_rounds)
            leave_combat_to_recover("fire_rounds_done")
            return
        fx_fire_wait_led()
        g_fire_phase = FIRE_PHASE_BURST_WAIT
        g_phase_t0 = now_s()
        log("FIRE wait %.1fs" % T_BURST_WAIT)
        return

    if g_fire_phase == FIRE_PHASE_BURST_WAIT:
        if phase_age() >= T_BURST_WAIT:
            if people_seen() == False:
                log("FIRE wait end no person -> RECOVER")
                leave_combat_to_recover("fire_wait_empty")
                return
            g_fire_rounds = g_fire_rounds + 1
            if g_fire_rounds > FIRE_MAX_ROUNDS:
                leave_combat_to_recover("fire_rounds_cap")
                return
            g_fire_phase = FIRE_PHASE_BURST_ON
            g_phase_t0 = now_s()
            fire_bead_burst_start()
        return

def tick_recover():
    fire_stop()
    chassis_halt()
    gimbal_stop()
    try:
        if get_pitch() > (PITCH_LINE + 8):
            gimbal_set_pitch_line()
    except Exception:
        gimbal_set_pitch_line()
    line_update()
    # 冷却期内不因人打断找线
    if person_cooldown_active() == False:
        if person_hit_update(PERSON_HIT_NEED_SCAN):
            set_state(STATE_LOCK, "person_on_recover")
            return
    if line_stable_true():
        log("RECOVER line -> PATROL")
        set_state(STATE_PATROL, "line_found")
        return
    # 找不到线：不要马上又 SCAN（会死循环）；多等一会再扫
    if line_stable_false() and state_age() >= 2.0:
        if g_scan_empty_count >= SCAN_EMPTY_MAX:
            # 继续低头找，不扫
            if state_age() >= 6.0:
                log("RECOVER still no line -> PATROL try follow")
                set_state(STATE_PATROL, "recover_give_patrol")
            return
        log("RECOVER no line -> SCAN")
        set_state(STATE_SCAN, "still_no_line")
        return

# =============================================================================
# ENTRY
# =============================================================================
def setup():
    log("setup begin")
    mode_ensure_free("setup")
    set_gimbal_speed(HOME_YAW_SPEED)
    gimbal_pose_line()
    time.sleep(0.15)
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
    line_pid_init()
    log("setup done v1.17.0 T_MOVE=%.1f spd=%.2f pulse=%s" % (
        T_MOVE, LINE_SPEED, str(FIRE_USE_PULSE)
    ))

def start():
    global g_state
    print("======== Line Guard start ========")
    print("# LINE_GUARD_VERSION=1.17.0 stamp=2026-08-04 17:10:00")
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
