# LINE_GUARD_VERSION=1.27.0 stamp=2026-08-05 00:10:00  (paste this whole file; check stamp matches latest)
# -*- coding: utf-8 -*-
# S1 Line Guard — 单文件粘贴进 App 实验室
#
# 权威行为规格（改本文件前必读，禁止与之冲突）：
#   仓库 docs/behavior-spec.md
# 新增额外能力须先确认并写入规格，再改代码。
#
# 摘要：
#   PATROL  贴线 T_MOVE，不认人
#   SCAN    45° 步进；hit≥3→LOCK；整圈无人有线→PATROL；无线→原地再 SCAN
#   LOCK/FIRE  当前框 PID；射3s/停火瞄3s；miss>3→立刻整圈 SCAN
#   RECOVER 有线→PATROL；无线→SCAN

# =============================================================================
# CONFIG — 可调参数
# =============================================================================
# --- 时间 / 认人门控（认人、丢失一律用连续次数）---
T_MOVE = 6.0                    # PATROL 贴线前进多久（秒）后进入 SCAN
PERSON_HIT_NEED = 3             # 连续 hit 次数 → 确认有人，进 LOCK
PERSON_MISS_NEED = 3            # 连续 miss 超过此值（miss > 3）→ 确认丢失，整圈 SCAN

# --- 开火框 ---
PERSON_FIRE_MIN_W = 0.10        # 允许 IR/水弹的最小框宽（归一化）
PERSON_FIRE_MIN_H = 0.16        # 允许 IR/水弹的最小框高（归一化）

# --- 云台俯仰（绝对角，约 -20~+35）---
PITCH_LINE = -20                # 巡线/找线低头姿态
PITCH_SCAN = 20                 # 扫人/锁目标时抬头姿态（注意：水弹硬件约>10°禁射）

# --- SCAN 几何与步进 ---
SCAN_CW = 180.0                 # 扫描大目标：顺时针绝对 yaw（度）
SCAN_CCW = -180.0               # 扫描大目标：逆时针绝对 yaw（度）
SCAN_YAW_SPEED = 540.0          # 扫描 yaw_ctrl 前 set_rotate_speed（°/s，近极限）
SCAN_STEP_DEG = 45.0            # 每步绝对角最大步进（度）
SCAN_LOOK_OPS = 5               # 每角位最少查人次数；无人则满此数转下一角
SCAN_LOOK_OPS_MAX = 8           # 末段有部分 hit 时最多查人次数（凑满 PERSON_HIT_NEED）
YAW_ARRIVE = 10.0               # 认为到达大段目标 yaw 的误差容限（度）
HOME_YAW_SPEED = 540.0          # 回中/摆姿态时的云台转速（°/s）
T_SCAN_MAX = 90.0               # 单次 SCAN 状态最长秒数，超时 → RECOVER

# --- PATROL 循线（官方 RmList + PID）---
LINE_SPEED = 0.30               # 底盘循线平移速度（m/s）
LINE_PID_KP = 330.0             # 线跟踪 PID 比例（官方示例 330）
LINE_PID_KI = 0.0               # 线跟踪 PID 积分
LINE_PID_KD = 28.0              # 线跟踪 PID 微分（官方示例 28）
LINE_GIMBAL_YAW_MAX = 300.0     # 循线时云台 yaw 速度输出限幅（°/s）
LINE_CX_DEADZONE = 0.02         # 线中心误差死区（|cx-0.5| 小于此不转）
LINE_CONFIRM_FRAMES = 3         # 判定「有线稳定」的连续有效帧数
LINE_LOST_FRAMES = 12           # 跟线中判定丢线的连续无效帧数（比确认更严）
LINE_LOOK_DOWN_DEG = 20         # 进 PATROL 相对低头度数（官方 down 20）

# --- 瞄准 PID（LOCK/FIRE 跟人体框）---
AIM_YAW_KP = 100.0              # 瞄准 yaw PID 比例
AIM_YAW_KI = 0.0                # 瞄准 yaw PID 积分
AIM_YAW_KD = 20.0               # 瞄准 yaw PID 微分
AIM_YAW_OUT_MAX = 150.0         # 瞄准 yaw 速度正常上限（°/s）
AIM_PITCH_KP = 70.0             # 瞄准 pitch PID 比例
AIM_PITCH_KI = 0.0              # 瞄准 pitch PID 积分
AIM_PITCH_KD = 16.0             # 瞄准 pitch PID 微分
AIM_PITCH_OUT_MAX = 50.0        # 瞄准 pitch 速度上限（°/s）
AIM_DEADZONE = 0.04             # 图像误差死区，小于此不输出速度
AIM_OK_ERR = 0.10               # 认为「大致对准」的误差阈值（PID 可停转）
AIM_ACQUIRE_ERR = 0.22          # |err_yaw|≥此值视为边缘大误差，换用加速上限
AIM_ACQUIRE_YAW_MAX = 220.0     # 边缘拉回时的 yaw 速度上限（°/s）
AIM_FIRE_MAX_ERR = 0.18         # 首次 IR/开火前：|x-0.5|、|y-0.5| 须≤此值
AIM_TRACK_FIRE_ERR = 0.28       # 交战中可射：允许略偏（人在动）

# --- 行人检出几何过滤（API 报人后再筛；对扶手等仍可能误放行）---
PERSON_MIN_W = 0.08             # 有效人体最小框宽
PERSON_MIN_H = 0.14             # 有效人体最小框高
PERSON_MIN_ASPECT = 1.15        # 最小高宽比 h/w（偏扁框拒绝）
PERSON_MAX_CY = 0.72            # 框中心 y 上限（过靠下拒绝）

# --- LOCK 接敌 / FIRE 射停交替（见 behavior-spec）---
T_AIM_BEFORE_IR = 0.5           # 进 LOCK 后至少瞄准这么久（秒）才允许 IR
T_LOCK_AIM_MAX = 6.0            # LOCK 接敌超时（秒）仍未居中可射 → 重扫
T_FIRE_ON = 3.0                 # FIRE 射击段时长（秒）：瞄准+射击
T_FIRE_OFF = 3.0                # FIRE 停火段时长（秒）：只瞄准不射击
FIRE_PULSE_INTERVAL = 0.18      # 射击段内两次 fire_once 最小间隔（秒）
FIRE_BEADS_PER_PULSE = 2        # 每次 fire_once 设定的弹数（1~8）
FIRE_USE_PULSE = True           # True=短间隔 fire_once；False=fire_continuous

ENABLE_FIRE = True              # False=不 IR/不水弹（只跟瞄走状态）
FORCE_NO_LINE = False           # True=强制当无线（调试用，跳过真实线识别）
FLASH_HZ = 3                    # 部分灯效闪烁频率（Hz）

# =============================================================================
# STATE — 状态机与全局变量
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
FIRE_PHASE_SHOOT = 2            # 射击段：瞄准+射
FIRE_PHASE_HOLD = 3             # 停火段：只瞄准

g_state = STATE_INIT
g_state_t0 = 0.0
g_person_miss = 0
g_person_hit = 0
g_patrol_line_t0 = 0.0
g_aim_t_prev = 0.0
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

# SCAN 队列与当前段
g_scan_queue = []
g_scan_qi = 0
g_scan_target_yaw = 0.0
g_scan_last_yaw = 0.0
g_scan_stuck = 0
g_scan_seg_name = ""
g_mode_tag = "free"
# SCAN 诊断：步进序号、本角位已做查人次数（固定操作计数，非 sleep）
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
    leds_set(200, 0, 255, rm_define.effect_flash, 6)
    sfx(rm_define.media_sound_recognize_success)

def fx_fire_ir_led():
    leds_set(255, 100, 0, rm_define.effect_flash, 7)

def fx_fire_burst_led():
    leds_set(255, 0, 0, rm_define.effect_flash, 8, gun_on=True)

def fx_fire_wait_led():
    leds_set(255, 140, 0, rm_define.effect_always_on, FLASH_HZ)

def fx_person_lost():
    leds_set(255, 140, 0, rm_define.effect_breath, FLASH_HZ)
    sfx(rm_define.media_sound_attacked)

def fx_recover():
    leds_set(255, 255, 255, rm_define.effect_flash, 2)
    sfx(rm_define.media_sound_solmization_1G)

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

def people_seen():
    ok, x, y, w, h = people_get_first()
    return ok

def person_hit_reset():
    global g_person_hit
    g_person_hit = 0

def person_hit_update(need):
    """连续 need 帧见到人则确认。"""
    global g_person_hit
    if people_seen():
        g_person_hit = g_person_hit + 1
    else:
        g_person_hit = 0
    return g_person_hit >= need

def person_track_update():
    """LOCK/FIRE：更新 miss；仅当前帧有效检出算有人。"""
    global g_person_miss
    ok, x, y, w, h = people_get_first()
    if ok:
        g_person_miss = 0
        return True
    g_person_miss = g_person_miss + 1
    return False

def person_confirmed_lost():
    """连续 miss 超过 PERSON_MISS_NEED（miss > 3）→ 整圈 SCAN。"""
    return g_person_miss > PERSON_MISS_NEED

def leave_combat_to_rescan(reason):
    """人员消失：停火，立即从 SCAN 起点重跑完整循环。"""
    fire_stop()
    person_hit_reset()
    pid_reset_aim()
    fx_person_lost()
    set_state(STATE_SCAN, "rescan_after_%s" % reason)

def leave_combat_to_recover(reason):
    """停火后去找线（无冷却，可立刻再被 SCAN 认人）。"""
    fire_stop()
    set_state(STATE_RECOVER, reason)

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

def aim_pid_towards_xy(x, y, dt):
    """瞄准：把人体框中心 (x,y) 拉向画面中心（PID；边缘大误差加速）。"""
    global g_iy, g_ey_prev, g_ip, g_ep_prev
    err_yaw = x - 0.5
    err_pitch = y - 0.5
    yaw_max = AIM_YAW_OUT_MAX
    pitch_max = AIM_PITCH_OUT_MAX
    # 画面边缘：提高 yaw 输出上限，尽快对准
    if abs(err_yaw) >= AIM_ACQUIRE_ERR:
        yaw_max = AIM_ACQUIRE_YAW_MAX
    yaw_spd, g_iy, g_ey_prev = pid_step(
        err_yaw, g_iy, g_ey_prev, AIM_YAW_KP, AIM_YAW_KI, AIM_YAW_KD, yaw_max, dt
    )
    pitch_spd, g_ip, g_ep_prev = pid_step(
        err_pitch, g_ip, g_ep_prev, AIM_PITCH_KP, AIM_PITCH_KI, AIM_PITCH_KD, pitch_max, dt
    )
    if abs(err_yaw) < AIM_OK_ERR and abs(err_pitch) < AIM_OK_ERR:
        gimbal_stop()
        return True
    try:
        gimbal_ctrl.rotate_with_speed(yaw_spd, -pitch_spd)
    except Exception:
        pass
    return False

def aim_pid_track():
    """瞄准步进：仅跟当前帧有效框；无检出则停转（无 coast）。"""
    dt = aim_pid_dt()
    ok, x, y, w, h = people_get_first()
    if ok:
        return aim_pid_towards_xy(x, y, dt), True
    gimbal_stop()
    return False, False

# =============================================================================
# LINE VISION — RmList 读蓝线（与官方示例下标一致）
# =============================================================================
def line_get_rmlist():
    raw = vision_ctrl.get_line_detection_info()
    try:
        return RmList(raw)
    except Exception:
        return raw

def line_read():
    """
    官方判定：len==42 且 [2]>=1 时 cx=[19]。
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
    if n == 42 and pts >= 1:
        try:
            cx = float(info[19])
            return True, cx, n, pts
        except Exception:
            return False, 0.5, n, pts
    if n >= 42 and pts >= 1:
        try:
            cx = float(info[19])
            return True, cx, n, pts
        except Exception:
            return False, 0.5, n, pts
    return False, 0.5, n, pts

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
    """初始化官方 PIDCtrl(330,0,28)。"""
    global g_line_pid
    g_line_pid = None
    try:
        g_line_pid = PIDCtrl()
        g_line_pid.set_ctrl_params(LINE_PID_KP, LINE_PID_KI, LINE_PID_KD)
        log("LINE PIDCtrl ok")
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
        log("LINE PIDCtrl unavailable, P only")

def line_look_down_official():
    """相对低头 LINE_LOOK_DOWN_DEG，对准地面蓝线。"""
    global g_line_look_done
    try:
        gimbal_ctrl.rotate_with_speed(0, 0)
    except Exception:
        pass
    try:
        gimbal_ctrl.yaw_ctrl(0)
    except Exception:
        pass
    try:
        gimbal_ctrl.pitch_ctrl(0)
    except Exception:
        pass
    try:
        gimbal_ctrl.rotate_with_degree(rm_define.gimbal_down, LINE_LOOK_DOWN_DEG)
        g_line_look_done = True
        log("LINE look_down relative %d deg" % LINE_LOOK_DOWN_DEG)
    except Exception:
        try:
            gimbal_ctrl.pitch_ctrl(-LINE_LOOK_DOWN_DEG)
            g_line_look_done = True
            log("LINE look_down fallback pitch=%d" % (-LINE_LOOK_DOWN_DEG))
        except Exception:
            log("LINE look_down FAIL")

# =============================================================================
# ACTUATORS — 云台姿态、模式、循线步进、射击
# =============================================================================
def gimbal_set_pitch_line():
    set_gimbal_speed(HOME_YAW_SPEED)
    try:
        gimbal_ctrl.pitch_ctrl(PITCH_LINE)
    except Exception:
        pass

def gimbal_set_pitch_scan():
    set_gimbal_speed(HOME_YAW_SPEED)
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
    set_gimbal_speed(HOME_YAW_SPEED)
    try:
        gimbal_ctrl.pitch_ctrl(PITCH_SCAN)
    except Exception:
        pass

def gimbal_pose_line():
    """巡线姿态：yaw=0，pitch 低头。"""
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
    """SCAN/LOCK/FIRE 等：停底盘 + robot_mode_free，云台独立。"""
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
    """PATROL：chassis_follow，云台带动底盘循线。"""
    global g_mode_tag
    robot_ctrl.set_mode(rm_define.robot_mode_chassis_follow)
    g_mode_tag = "chassis_follow"
    log("MODE chassis_follow | %s" % reason)

def line_follow_step():
    """单步循线：PID 云台 yaw + 底盘前进。"""
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
    """当前帧有人体且框够大，允许 IR/水弹。"""
    ok, x, y, w, h = people_get_first()
    if ok == False:
        return False
    try:
        if w < PERSON_FIRE_MIN_W or h < PERSON_FIRE_MIN_H:
            return False
    except Exception:
        return False
    return True

def person_aim_err():
    """当前人体相对画面中心误差 (ey, ep)。无有效检出返回 (1,1)。"""
    ok, x, y, w, h = people_get_first()
    if ok == False:
        return 1.0, 1.0
    return abs(x - 0.5), abs(y - 0.5)

def person_aim_centered():
    """首次开火门：当前框大致对准。"""
    ey, ep = person_aim_err()
    if ey > AIM_FIRE_MAX_ERR:
        return False
    if ep > AIM_FIRE_MAX_ERR:
        return False
    return True

def person_aim_track_ok():
    """交战中可射：当前框略偏仍可（须本帧有效检出）。"""
    ok, x, y, w, h = people_get_first()
    if ok == False:
        return False
    ey, ep = person_aim_err()
    if ey > AIM_TRACK_FIRE_ERR:
        return False
    if ep > AIM_TRACK_FIRE_ERR:
        return False
    return True

def fire_ir_warn_once():
    """红外示警一次。"""
    global g_fire_count, g_ir_done
    if g_ir_done:
        return
    if person_fire_ok() == False:
        log("IR_WARN skip no solid person")
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
        gun_ctrl.set_fire_count(1)
        gun_ctrl.fire_once()
        log("IR_WARN fallback gun")
    g_fire_count = g_fire_count + 1
    g_ir_done = True

def fire_bead_burst_start():
    """射击段开始：脉冲 fire_once 或 continuous（须当前帧可射）。"""
    global g_last_shot_t, g_burst_shots, g_fire_count
    if ENABLE_FIRE == False:
        log("FIRE_ON skip ENABLE_FIRE=0")
        return
    if person_fire_ok() == False:
        log("FIRE_ON skip no solid person miss=%d" % g_person_miss)
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
            log("FIRE_ON continuous %.1fs" % T_FIRE_ON)
        except Exception:
            log("FIRE continuous FAIL")
        return
    fire_bead_pulse_once()
    log("FIRE_ON pulse interval=%.2fs for %.1fs" % (FIRE_PULSE_INTERVAL, T_FIRE_ON))

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
        log("FIRE pulse #%d" % g_burst_shots)
    except Exception:
        log("FIRE pulse fail")

def fire_bead_burst_tick():
    """射击段内补发；须当前帧 person_fire_ok。"""
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
    """设定极限转速后绝对 yaw_ctrl 到 tgt。返回 (yaw0, yaw1, dt_s)。"""
    gimbal_stop()
    set_gimbal_speed(SCAN_YAW_SPEED)
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
        y0, tgt, y1, SCAN_YAW_SPEED, dt, reason
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
    set_gimbal_speed(HOME_YAW_SPEED)
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

def scan_load_segment(qi):
    """装载第 qi 段大目标角。"""
    global g_scan_qi, g_scan_target_yaw, g_scan_last_yaw, g_scan_stuck, g_scan_seg_name
    g_scan_qi = qi
    g_scan_target_yaw = g_scan_queue[qi]
    g_scan_last_yaw = get_yaw()
    g_scan_stuck = 0
    g_scan_seg_name = scan_seg_label(qi, g_scan_target_yaw)
    if abs(g_scan_target_yaw) < 0.1:
        fx_recenter()
    else:
        fx_scan_seg(g_scan_seg_name)
    log("SCAN seg qi=%d %s yaw0=%.0f" % (qi, g_scan_seg_name, g_scan_last_yaw))

def scan_diag_reset_cycle():
    """新 SCAN 循环：清诊断计数。"""
    global g_scan_step_i, g_scan_look_ops, g_scan_cycle_look_ops
    global g_scan_cycle_turns, g_scan_t_after_turn
    g_scan_step_i = 0
    g_scan_look_ops = 0
    g_scan_cycle_look_ops = 0
    g_scan_cycle_turns = 0
    g_scan_t_after_turn = now_s()

def scan_start_full(reason):
    """从回中开始完整 SCAN 循环。"""
    global g_scan_queue
    mode_ensure_free("scan_start_full")
    g_scan_queue = scan_queue_full_two_rounds()
    person_hit_reset()
    scan_diag_reset_cycle()
    set_gimbal_speed(HOME_YAW_SPEED)
    try:
        gimbal_ctrl.angle_ctrl(0, PITCH_SCAN)
    except Exception:
        try:
            gimbal_ctrl.yaw_ctrl(0)
        except Exception:
            pass
        gimbal_set_pitch_scan()
    gimbal_set_pitch_scan()
    g_scan_t_after_turn = now_s()
    log("SCAN home first yaw=%.0f pitch=%.0f" % (get_yaw(), get_pitch()))
    if len(g_scan_queue) <= 0:
        log("SCAN empty queue")
        return
    fx_scan()
    scan_load_segment(0)
    log(
        "SCAN plan: home -> CW%+.0f -> 0 -> CCW%+.0f -> 0 | step=%.0f look=%d..%d hit_need=%d | %s"
        % (SCAN_CW, SCAN_CCW, SCAN_STEP_DEG, SCAN_LOOK_OPS, SCAN_LOOK_OPS_MAX, PERSON_HIT_NEED, reason)
    )

def scan_look_once():
    """
    一次固定「查人」操作：单次视觉采样 + 更新 hit。
    不用 sleep 凑时间；dt_op = 本次采样实际耗时。
    返回 (locked, person_seen, dt_op)
    """
    global g_scan_look_ops, g_scan_cycle_look_ops
    t0 = now_s()
    # person_hit_update 内部只采样一次；hit 即当前累计
    locked = person_hit_update(PERSON_HIT_NEED)
    dt = now_s() - t0
    seen = g_person_hit > 0
    g_scan_look_ops = g_scan_look_ops + 1
    g_scan_cycle_look_ops = g_scan_cycle_look_ops + 1
    # gap_since_turn：距上次转完的墙钟，仅诊断（含主循环 sleep，非我们主动空等）
    gap = 0.0
    if g_scan_t_after_turn > 0.0:
        gap = t0 - g_scan_t_after_turn
    # 有效检出时带上框尺寸，便于区分真人 / 行李误检
    whs = ""
    if seen:
        ok2, px, py, pw, ph = people_get_first()
        if ok2:
            whs = " xy=(%.2f,%.2f) wh=(%.2f,%.2f)" % (px, py, pw, ph)
    log(
        "SCAN_LOOK step=%d yaw=%.0f seg=%s look_ops=%d hit=%d/%d person=%s "
        "dt_op=%.3f gap_since_turn=%.3f locked=%s%s"
        % (
            g_scan_step_i, get_yaw(), g_scan_seg_name, g_scan_look_ops,
            g_person_hit, PERSON_HIT_NEED, str(seen), dt, gap, str(locked), whs
        )
    )
    return locked, seen, dt

def scan_tick_turn():
    """
    朝本段大目标走一步 SCAN_STEP_DEG（绝对 yaw_ctrl）。
    返回 (seg_done, did_turn)
      seg_done: 本段大目标已到
      did_turn: 本 tick 是否执行了 yaw_ctrl
    """
    global g_scan_step_i, g_scan_look_ops, g_scan_cycle_turns, g_scan_t_after_turn
    tgt = g_scan_target_yaw
    yaw0 = get_yaw()
    err = tgt - yaw0
    if abs(err) <= YAW_ARRIVE:
        log(
            "SCAN_SEG_ARRIVE step=%d yaw=%.0f tgt=%.0f look_ops_at_angle=%d | %s"
            % (g_scan_step_i, yaw0, tgt, g_scan_look_ops, g_scan_seg_name)
        )
        return True, False
    step = SCAN_STEP_DEG
    if step < 5.0:
        step = 5.0
    if err > 0:
        if err > step:
            nxt = yaw0 + step
        else:
            nxt = tgt
    else:
        if err < -step:
            nxt = yaw0 - step
        else:
            nxt = tgt
    if nxt > 250.0:
        nxt = 250.0
    if nxt < -250.0:
        nxt = -250.0
    # 转之前应已完成 SCAN_LOOK_OPS 次查人（由 tick 保证）
    look_before = g_scan_look_ops
    y0, y1, dt_turn = scan_yaw_abs(nxt, "%s step%.0f" % (g_scan_seg_name, step))
    g_scan_cycle_turns = g_scan_cycle_turns + 1
    g_scan_step_i = g_scan_step_i + 1
    g_scan_t_after_turn = now_s()
    log(
        "SCAN_TURN step=%d yaw %.0f->%.0f plan_nxt=%.0f err0=%.0f "
        "dt_turn=%.3f look_ops_before=%d need=%d seg=%s"
        % (
            g_scan_step_i, y0, y1, nxt, err, dt_turn, look_before, SCAN_LOOK_OPS, g_scan_seg_name
        )
    )
    # 新角位：查人计数从 0 重新计
    g_scan_look_ops = 0
    if abs(tgt - y1) <= YAW_ARRIVE:
        log(
            "SCAN_SEG_ARRIVE step=%d yaw=%.0f tgt=%.0f | %s"
            % (g_scan_step_i, y1, tgt, g_scan_seg_name)
        )
        return True, True
    return False, True

def scan_advance_or_finish():
    """本段完成：下一段或整圈结束。下一段从 look_ops=0 再观测。"""
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
    """
    状态切换：
      非 PATROL → free 模式
      PATROL → chassis_follow + 低头循线
    """
    global g_state, g_state_t0, g_person_miss
    global g_patrol_line_t0, g_fire_count, g_fire_phase, g_phase_t0
    global g_ir_done, g_line_hit, g_line_miss, g_line_ever_ok
    old = g_state
    g_state = s
    g_state_t0 = now_s()
    if s != STATE_FIRE:
        g_person_miss = 0
    log("STATE %s -> %s | %s" % (state_name(old), state_name(s), reason))

    if s != STATE_PATROL:
        fire_stop()
        mode_ensure_free("enter_%s" % state_name(s))

    # PATROL：官方循线准备
    if s == STATE_PATROL:
        fire_stop()
        mode_ensure_free("patrol_before_pose")
        try:
            gimbal_ctrl.yaw_ctrl(0)
            gimbal_ctrl.pitch_ctrl(0)
        except Exception:
            pass
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
        log("PATROL ready spd=%.2f" % LINE_SPEED)

    # SCAN：完整两遍步进扫描
    if s == STATE_SCAN:
        scan_start_full("normal_scan")

    # LOST_SCAN：同完整 SCAN
    if s == STATE_LOST_SCAN:
        pid_reset_aim()
        fx_person_lost()
        mode_ensure_free("lost_before_scan")
        scan_start_full("lost_rescan_full")

    # LOCK = 瞄准：当前框 PID，居中后再 IR → FIRE
    if s == STATE_LOCK:
        gimbal_ensure_pitch_scan_soft()
        pid_reset_aim()
        person_hit_reset()
        fx_lock()
        g_fire_count = 0
        g_ir_done = False
        g_fire_phase = FIRE_PHASE_AIM
        g_phase_t0 = now_s()
        g_person_miss = 0
        ok, x, y, w, h = people_get_first()
        if ok == False:
            x = 0.5
            y = 0.5
            w = 0.0
            h = 0.0
        log("LOCK target ok=%s xy=(%.2f,%.2f) wh=(%.2f,%.2f)" % (
            str(ok), x, y, w, h
        ))

    # FIRE：射 3s / 停火瞄 3s
    if s == STATE_FIRE:
        mode_ensure_free("enter_FIRE")
        pid_reset_aim()
        if g_ir_done:
            g_fire_phase = FIRE_PHASE_IR_DONE
        else:
            g_fire_phase = FIRE_PHASE_AIM
            fx_lock()
        g_phase_t0 = now_s()
        g_person_miss = 0
        log("FIRE enter ir_done=%s on=%.1fs off=%.1fs" % (
            str(g_ir_done), T_FIRE_ON, T_FIRE_OFF
        ))

    # RECOVER：低头找线
    if s == STATE_RECOVER:
        pid_reset_aim()
        gimbal_pose_line()
        fx_recover()
        g_line_hit = 0
        g_line_miss = 0
        person_hit_reset()
        log("RECOVER find line pitch=%.0f" % get_pitch())

def tick_patrol():
    """蓝线巡线 T_MOVE 秒后进 SCAN；巡线阶段不认人、不进 LOCK。"""
    global g_patrol_line_t0
    line_update()
    if line_stable_false():
        if g_line_ever_ok:
            g_patrol_line_t0 = 0.0
            person_hit_reset()
            set_state(STATE_SCAN, "no_line_stable")
            return
        if state_age() >= 4.0:
            log("PATROL no line ever -> SCAN")
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
    if g_patrol_line_t0 <= 0.0:
        g_patrol_line_t0 = now_s()
        mode_ensure_line_follow("patrol_follow_begin")
        log("PATROL follow start cx=%.2f pts=%d" % (g_line_cx, g_line_pts))
    line_follow_step()
    if (now_s() - g_patrol_line_t0) >= T_MOVE:
        age = now_s() - g_patrol_line_t0
        g_patrol_line_t0 = 0.0
        person_hit_reset()
        log("PATROL follow done age=%.2fs -> SCAN" % age)
        set_state(STATE_SCAN, "follow_time_up")

def scan_look_should_keep():
    """
    本角位是否继续查人（不转）：
      look < 5：继续
      look 已满 5 且 hit 在 (0, need) 之间：再延，最多到 8
      look >= 8 或 hit==0 且已满 5：不再查，该转
    """
    need = PERSON_HIT_NEED
    if need < 1:
        need = 1
    lo_min = SCAN_LOOK_OPS
    if lo_min < 1:
        lo_min = 1
    lo_max = SCAN_LOOK_OPS_MAX
    if lo_max < lo_min:
        lo_max = lo_min
    if g_scan_look_ops < lo_min:
        return True
    if g_scan_look_ops >= lo_max:
        return False
    # 5 <= look < 8：若已有部分连续 hit，再等几帧凑满 need
    if g_person_hit > 0 and g_person_hit < need:
        return True
    return False

def tick_scan_common():
    """
    SCAN 调度（固定操作，无 sleep）：
      LOOK：每 tick 1 次查人
            任意时刻 hit 连续 >=3 → 立刻 LOCK（可第 3 次就进，不等满 5）
            无人：满 5 次转下一角
            满 5 时 0<hit<3：再延到最多 8 次
      TURN：本角放弃 → 45° 下一步
      整圈无人：有线 → PATROL；无线 → 原地再 SCAN（见 behavior-spec）
    """
    global g_scan_look_ops
    chassis_halt()

    # 超时保护：回中找线；无线仍会再 SCAN
    if state_age() >= T_SCAN_MAX:
        log("SCAN timeout age=%.1f -> RECOVER" % state_age())
        scan_log_cycle_summary("timeout")
        gimbal_stop()
        set_state(STATE_RECOVER, "scan_timeout")
        return

    # ----- LOOK 相位：先查人；够 hit 立即 LOCK，与是否满 5 无关 -----
    if scan_look_should_keep():
        locked, seen, dt_look = scan_look_once()
        if locked:
            # 例：头 3 次全 hit → look_ops=3 即进，不会拖到 5
            gimbal_stop()
            log(
                "SCAN person LOCK early_ok step=%d look_ops=%d hit=%d need=%d (min_look=%d)"
                % (
                    g_scan_step_i, g_scan_look_ops, g_person_hit, PERSON_HIT_NEED,
                    SCAN_LOOK_OPS
                )
            )
            set_state(STATE_LOCK, "person_on_scan")
            return
        # 未 LOCK：未满 5 / 部分 hit 需延 → 继续 LOOK；否则 TURN
        if scan_look_should_keep():
            return

    # ----- TURN 相位：本角观测结束仍未 LOCK -----
    if g_person_hit > 0:
        log(
            "SCAN look give_up step=%d look_ops=%d hit=%d need=%d (max=%d) -> turn"
            % (g_scan_step_i, g_scan_look_ops, g_person_hit, PERSON_HIT_NEED, SCAN_LOOK_OPS_MAX)
        )
    person_hit_reset()
    seg_done, did_turn = scan_tick_turn()
    if did_turn == False and seg_done == False:
        return
    if seg_done == False:
        # 已转到新角，look_ops 已在 scan_tick_turn 清 0
        return

    # 本段大目标已到（可能本 tick 刚转到位，或原本已在目标）
    log("SCAN seg done %s yaw=%.0f step=%d" % (g_scan_seg_name, get_yaw(), g_scan_step_i))
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

def tick_lost_scan():
    tick_scan_common()

def tick_lock():
    """
    锁目标·接敌：当前框 PID；可射 → IR → FIRE。
    miss 超过 3 → 整圈 SCAN。
    """
    global g_fire_phase, g_phase_t0
    chassis_halt()
    person_track_update()
    aim_pid_track()

    if person_confirmed_lost():
        log("LOCK lost miss=%d -> SCAN" % g_person_miss)
        leave_combat_to_rescan("lock_lost")
        return

    if g_ir_done == False and state_age() >= T_LOCK_AIM_MAX:
        ok, x, y, w, h = people_get_first()
        log("LOCK acquire timeout xy=(%.2f,%.2f) -> rescan" % (x, y))
        leave_combat_to_rescan("lock_timeout")
        return

    if g_ir_done == False and state_age() >= T_AIM_BEFORE_IR:
        if person_aim_centered() == False:
            return
        if person_fire_ok() == False:
            return
        fire_ir_warn_once()
        if g_ir_done:
            g_fire_phase = FIRE_PHASE_IR_DONE
            g_phase_t0 = now_s()
            ok, x, y, w, h = people_get_first()
            log("LOCK engage xy=(%.2f,%.2f) -> FIRE" % (x, y))
            set_state(STATE_FIRE, "engage")

def tick_fire():
    """
    锁目标·交战（behavior-spec）：
      始终当前框 PID；
      射击段 T_FIRE_ON：瞄准+射；
      停火段 T_FIRE_OFF：只瞄准；
      miss>3 → 立刻 SCAN。
    """
    global g_fire_phase, g_phase_t0
    chassis_halt()
    person_track_update()

    if person_confirmed_lost():
        log("FIRE lost miss=%d -> SCAN" % g_person_miss)
        leave_combat_to_rescan("fire_lost")
        return

    aim_pid_track()

    # 尚未 IR
    if g_ir_done == False:
        if state_age() >= T_AIM_BEFORE_IR:
            if person_fire_ok() == False:
                return
            if person_aim_centered() == False and person_aim_track_ok() == False:
                return
            fire_ir_warn_once()
            if g_ir_done == False:
                return
            g_fire_phase = FIRE_PHASE_SHOOT
            g_phase_t0 = now_s()
            fire_bead_burst_start()
        return

    # IR 刚完成：进入射击段
    if g_fire_phase == FIRE_PHASE_IR_DONE:
        if person_fire_ok() == False:
            return
        if person_aim_track_ok() == False:
            return
        g_fire_phase = FIRE_PHASE_SHOOT
        g_phase_t0 = now_s()
        fire_bead_burst_start()
        return

    # 射击段 3s
    if g_fire_phase == FIRE_PHASE_SHOOT:
        if phase_age() < T_FIRE_ON:
            fire_bead_burst_tick()
            return
        fire_bead_burst_stop()
        fx_fire_wait_led()
        g_fire_phase = FIRE_PHASE_HOLD
        g_phase_t0 = now_s()
        log("FIRE hold aim-only %.1fs" % T_FIRE_OFF)
        return

    # 停火段 3s：只瞄准（上面已 aim_pid_track）
    if g_fire_phase == FIRE_PHASE_HOLD:
        if phase_age() >= T_FIRE_OFF:
            g_fire_phase = FIRE_PHASE_SHOOT
            g_phase_t0 = now_s()
            fire_bead_burst_start()
            log("FIRE shoot again %.1fs" % T_FIRE_ON)
        return

def tick_recover():
    """低头找线；有线→PATROL；无线→原地 SCAN（反复，见 behavior-spec）。"""
    fire_stop()
    chassis_halt()
    gimbal_stop()
    try:
        if get_pitch() > (PITCH_LINE + 8):
            gimbal_set_pitch_line()
    except Exception:
        gimbal_set_pitch_line()
    line_update()
    if person_hit_update(PERSON_HIT_NEED):
        set_state(STATE_LOCK, "person_on_recover")
        return
    if line_stable_true():
        log("RECOVER line -> PATROL")
        set_state(STATE_PATROL, "line_found")
        return
    if line_stable_false() and state_age() >= 2.0:
        log("RECOVER no line -> SCAN (repeat until line)")
        set_state(STATE_SCAN, "recover_no_line")
        return

# =============================================================================
# ENTRY — 启动与主循环
# =============================================================================
def setup():
    log("setup begin")
    mode_ensure_free("setup")
    set_gimbal_speed(HOME_YAW_SPEED)
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
    line_pid_init()
    log("setup done v1.27.0 fire_on=%.1f fire_off=%.1f hit=%d miss>%d" % (
        T_FIRE_ON, T_FIRE_OFF, PERSON_HIT_NEED, PERSON_MISS_NEED
    ))

def start():
    global g_state
    print("======== Line Guard start ========")
    print("# LINE_GUARD_VERSION=1.27.0 stamp=2026-08-05 00:10:00")
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
