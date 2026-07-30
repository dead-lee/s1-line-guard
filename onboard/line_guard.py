# -*- coding: utf-8 -*-
"""
S1 Line Guard v1 — 沿线哨兵（单文件，整段粘贴进 App 实验室 Python）

功能概要：
  PATROL  低头沿蓝线走一段时间
  SCAN    停车抬头，云台左右扫人
  LOCK    发现人：红闪 + PID 把枪口对准人
  FIRE    对准后满 3s 仍看见人：每 1s 点射（无弹也可测逻辑）
  RECOVER 人离开：停火、回巡线俯仰、找线

使用：
  1. 新建 DIY → 选 Python
  2. 全选本文件粘贴 → 连接 S1 → 运行
  3. 场地：浅色地面 + 蓝色胶带单环；先空弹测逻辑

若某 API 报错：把完整报错发回，常见是枚举名/pid 对象名差异。
语音告警本期不做。
"""

# =============================================================================
# 1. 可调参数（CONFIG）
# =============================================================================

# --- 时间节奏 ---
T_MOVE = 2.0              # 每次低头循线时长（秒）
T_SCAN = 4.0              # 单次哨位扫描总时长（秒）
T_WARN_BEFORE_FIRE = 3.0  # 锁定后首次点射前等待（秒）
T_FIRE_INTERVAL = 1.0     # 点射间隔（秒）
T_CLEAR = 2.0             # 连续多久看不到人算离开（秒）
LOOP_DT = 0.05            # 主循环周期（秒）

# --- 云台姿态（实车请微调）---
PITCH_LINE = -20          # 巡线俯仰（低头看线）
PITCH_SCAN = 5            # 扫人俯仰（略抬头）
YAW_SCAN_MIN = -70        # 扫描 yaw 左限
YAW_SCAN_MAX = 70         # 扫描 yaw 右限
YAW_SCAN_SPEED = 60       # 扫描角速度 °/s 量级（经 rotate 相对角实现）

# --- 循线 ---
LINE_SPEED = 0.35         # 巡线前进速度 m/s
LINE_PID_KP = 80.0        # 循线转向 P（轮速差）
LINE_PID_OUT_MAX = 80.0   # 轮速修正限幅

# --- 瞄准 PID（误差 = 目标中心相对画面中心，约 -0.5~0.5）---
AIM_YAW_KP = 180.0
AIM_YAW_KI = 0.0
AIM_YAW_KD = 8.0
AIM_YAW_OUT_MAX = 120.0   # 云台 yaw 速度限幅 °/s

AIM_PITCH_KP = 120.0
AIM_PITCH_KI = 0.0
AIM_PITCH_KD = 6.0
AIM_PITCH_OUT_MAX = 80.0

# 认为“已对准”的误差阈值（归一化画面坐标）
AIM_OK_ERR = 0.08

# --- 开关 ---
ENABLE_FIRE = True        # 无弹也可 True，只测开火逻辑
ENABLE_LINE = True        # False 时 PATROL 改为原地等待（方便无胶带时测扫描/锁定）

# --- 灯 ---
FLASH_HZ = 4


# =============================================================================
# 2. 状态与全局变量
# =============================================================================

STATE_INIT = 0
STATE_PATROL = 1
STATE_SCAN = 2
STATE_LOCK = 3
STATE_FIRE = 4
STATE_RECOVER = 5

g_state = STATE_INIT
g_state_t0 = 0.0          # 进入当前状态时的程序运行时间
g_no_person_t0 = 0.0      # 开始连续丢人的时间
g_last_fire_t = 0.0
g_scan_dir = 1            # 扫描方向 +1 / -1
g_scan_yaw = 0.0

# 软件 PID 状态：yaw
g_iy = 0.0
g_ey_prev = 0.0
# pitch
g_ip = 0.0
g_ep_prev = 0.0


# =============================================================================
# 3. 工具：时间 / 限幅 / PID
# =============================================================================

def now_s():
    return tools.run_time_of_program()


def clamp(v, lo, hi):
    if v < lo:
        return lo
    if v > hi:
        return hi
    return v


def pid_reset_aim():
    global g_iy, g_ey_prev, g_ip, g_ep_prev
    g_iy = 0.0
    g_ey_prev = 0.0
    g_ip = 0.0
    g_ep_prev = 0.0


def pid_step(err, i_acc, e_prev, kp, ki, kd, out_max, dt):
    """返回 (output, new_i, new_e_prev)"""
    i_new = i_acc + err * dt
    # 简单抗饱和
    i_new = clamp(i_new, -2.0, 2.0)
    if dt > 0.0001:
        d = (err - e_prev) / dt
    else:
        d = 0.0
    out = kp * err + ki * i_new + kd * d
    out = clamp(out, -out_max, out_max)
    return out, i_new, err


# =============================================================================
# 4. 灯效
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
# 5. 行人检测
# =============================================================================

def people_get_first():
    """
    返回 (ok, x, y, w, h)
    x,y 为行人中心在视野中的归一化坐标，约 0~1，画面中心约 0.5
    """
    info = vision_ctrl.get_people_detection_info()
    # 兼容 list / RmList：第 1 项数量 N，随后每组 X,Y,W,H
    try:
        n = info[0]
    except Exception:
        return False, 0.5, 0.5, 0.0, 0.0
    if n is None or n < 1:
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


# =============================================================================
# 6. 云台：巡线俯仰 / 扫描 / PID 瞄准
# =============================================================================

def gimbal_set_pitch_line():
    gimbal_ctrl.pitch_ctrl(PITCH_LINE)


def gimbal_set_pitch_scan():
    gimbal_ctrl.pitch_ctrl(PITCH_SCAN)


def gimbal_stop():
    gimbal_ctrl.stop()


def aim_pid_towards_person(dt):
    """
    用 PID 驱动云台，使人中心靠近画面中心。
    返回 (aligned, has_person)
    """
    global g_iy, g_ey_prev, g_ip, g_ep_prev
    ok, x, y, w, h = people_get_first()
    if not ok:
        gimbal_stop()
        return False, False

    # 误差：目标在右侧 x>0.5 → 需要云台右转（正 yaw 速度）
    err_yaw = x - 0.5
    # y 向下增大时：人偏下 y>0.5 → 需要低头（负 pitch 速度）
    err_pitch = y - 0.5

    yaw_spd, g_iy, g_ey_prev = pid_step(
        err_yaw, g_iy, g_ey_prev,
        AIM_YAW_KP, AIM_YAW_KI, AIM_YAW_KD, AIM_YAW_OUT_MAX, dt
    )
    pitch_spd, g_ip, g_ep_prev = pid_step(
        err_pitch, g_ip, g_ep_prev,
        AIM_PITCH_KP, AIM_PITCH_KI, AIM_PITCH_KD, AIM_PITCH_OUT_MAX, dt
    )

    # 官方：yaw 正=右，pitch 正=上；人偏下要负 pitch
    gimbal_ctrl.rotate_with_speed(yaw_spd, -pitch_spd)

    aligned = (abs(err_yaw) < AIM_OK_ERR) and (abs(err_pitch) < AIM_OK_ERR)
    return aligned, True


# =============================================================================
# 7. 底盘 / 循线
# =============================================================================

def chassis_halt():
    chassis_ctrl.stop()


def line_follow_step():
    """
    根据线识别信息做简单 P 循线。
    线信息格式因固件略有差异：常见 [.., 中心 x 在 index 19 附近]
    解析失败则直行慢速。
    """
    if not ENABLE_LINE:
        chassis_halt()
        return

    info = vision_ctrl.get_line_detection_info()
    # 默认直行
    vx = LINE_SPEED
    yaw_rate = 0.0

    try:
        # 社区常用：长度约 42 且 info[2]>=1 表示看到线；info[19] 为线中心 x
        n = 0
        try:
            n = len(info)
        except Exception:
            n = 0
        if n >= 20:
            # 尝试：第 3 项是否“有线”
            has = True
            try:
                if info[2] is not None and info[2] < 1:
                    has = False
            except Exception:
                has = True
            if has:
                cx = info[19]
                err = cx - 0.5
                # 线偏右 → 车向右转（正旋转）还是左？麦克纳姆常见：
                # move_with_speed(x, y, z) z 为旋转；经验上 err 取反需实车反馈
                yaw_rate = clamp(-err * LINE_PID_KP, -LINE_PID_OUT_MAX, LINE_PID_OUT_MAX)
    except Exception:
        yaw_rate = 0.0

    chassis_ctrl.move_with_speed(vx, 0, yaw_rate)


# =============================================================================
# 8. 射击
# =============================================================================

def fire_once_safe():
    if ENABLE_FIRE:
        gun_ctrl.set_fire_count(1)
        gun_ctrl.fire_once()


def fire_stop():
    gun_ctrl.stop()


# =============================================================================
# 9. 状态切换
# =============================================================================

def set_state(s):
    global g_state, g_state_t0, g_no_person_t0, g_last_fire_t
    global g_scan_dir, g_scan_yaw
    g_state = s
    g_state_t0 = now_s()
    g_no_person_t0 = now_s()

    if s == STATE_PATROL:
        fire_stop()
        chassis_halt()
        robot_ctrl.set_mode(rm_define.robot_mode_free)
        gimbal_set_pitch_line()
        leds_normal()
        # 巡线时也可顺便开线识别
        vision_ctrl.enable_detection(rm_define.vision_detection_line)

    elif s == STATE_SCAN:
        fire_stop()
        chassis_halt()
        robot_ctrl.set_mode(rm_define.robot_mode_free)
        gimbal_set_pitch_scan()
        g_scan_dir = 1
        g_scan_yaw = 0.0
        # 扫人时仍保留线识别，恢复时更快；行人已在 init 打开
        leds_normal()

    elif s == STATE_LOCK:
        chassis_halt()
        robot_ctrl.set_mode(rm_define.robot_mode_free)
        gimbal_set_pitch_scan()
        pid_reset_aim()
        leds_alert_red()
        g_no_person_t0 = now_s()

    elif s == STATE_FIRE:
        chassis_halt()
        pid_reset_aim()
        leds_alert_red()
        g_last_fire_t = 0.0  # 进入后马上允许第一发（满足 3s 后）

    elif s == STATE_RECOVER:
        fire_stop()
        chassis_halt()
        gimbal_stop()
        gimbal_set_pitch_line()
        leds_normal()


def state_age():
    return now_s() - g_state_t0


# =============================================================================
# 10. 各状态 tick
# =============================================================================

def tick_patrol():
    # 循线过程中若已看到人，直接锁定（可选增强）
    if people_seen():
        set_state(STATE_LOCK)
        return

    line_follow_step()

    if state_age() >= T_MOVE:
        set_state(STATE_SCAN)


def tick_scan():
    global g_scan_dir, g_scan_yaw

    chassis_halt()

    if people_seen():
        set_state(STATE_LOCK)
        return

    # 简单左右扫：用相对小步旋转模拟扫描
    # 使用 rotate_with_speed 持续转，到边界反向
    step = YAW_SCAN_SPEED * LOOP_DT * g_scan_dir
    g_scan_yaw = g_scan_yaw + step
    if g_scan_yaw >= YAW_SCAN_MAX:
        g_scan_yaw = YAW_SCAN_MAX
        g_scan_dir = -1
    if g_scan_yaw <= YAW_SCAN_MIN:
        g_scan_yaw = YAW_SCAN_MIN
        g_scan_dir = 1

    gimbal_ctrl.rotate_with_speed(YAW_SCAN_SPEED * g_scan_dir, 0)

    if state_age() >= T_SCAN:
        gimbal_stop()
        set_state(STATE_PATROL)


def tick_lock():
    global g_no_person_t0

    chassis_halt()
    aligned, has = aim_pid_towards_person(LOOP_DT)

    if not has:
        # 丢人计时
        if now_s() - g_no_person_t0 >= T_CLEAR:
            set_state(STATE_RECOVER)
        return

    # 仍看见人，刷新丢人计时
    g_no_person_t0 = now_s()

    # 锁定满 3s → 开火（对准过程中也计时；更严可要求 aligned）
    if state_age() >= T_WARN_BEFORE_FIRE:
        set_state(STATE_FIRE)


def tick_fire():
    global g_no_person_t0, g_last_fire_t

    chassis_halt()
    aligned, has = aim_pid_towards_person(LOOP_DT)

    if not has:
        if now_s() - g_no_person_t0 >= T_CLEAR:
            fire_stop()
            set_state(STATE_RECOVER)
        return

    g_no_person_t0 = now_s()

    # 持续 PID 对准的同时按间隔点射
    t = now_s()
    if g_last_fire_t <= 0.0 or (t - g_last_fire_t) >= T_FIRE_INTERVAL:
        # 可选：仅对准较好时开火，减少乱射
        if aligned or True:
            fire_once_safe()
            g_last_fire_t = t


def tick_recover():
    chassis_halt()
    gimbal_set_pitch_line()
    # 短暂停顿后回巡逻
    if state_age() >= 0.8:
        set_state(STATE_PATROL)


# =============================================================================
# 11. 初始化与主循环
# =============================================================================

def setup():
    robot_ctrl.set_mode(rm_define.robot_mode_free)
    chassis_halt()
    gimbal_ctrl.recenter()
    time.sleep(0.3)

    # 视觉
    vision_ctrl.enable_detection(rm_define.vision_detection_people)
    vision_ctrl.enable_detection(rm_define.vision_detection_line)
    vision_ctrl.line_follow_color_set(rm_define.line_follow_color_blue)
    # 循线曝光可按场地改：small 更不易糊
    media_ctrl.exposure_value_update(rm_define.exposure_value_medium)

    gun_ctrl.set_fire_count(1)
    leds_normal()
    gimbal_set_pitch_line()


def start():
    """App 实验室入口"""
    global g_state
    setup()
    set_state(STATE_PATROL)

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
            set_state(STATE_PATROL)

        time.sleep(LOOP_DT)
