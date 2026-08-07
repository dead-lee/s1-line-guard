# 设计备忘

## 权威行为规格

**状态机与已定业务规则以 [`behavior-spec.md`](./behavior-spec.md) 为准。**

改 `onboard/line_guard.py` 前必须对照该文档。

---

## 机身灯光含义（测试对照）

装甲灯 RGB + 效果；与 `line_guard.py` 中 `fx_*` 一致。

| 观感 | 效果 | 含义 |
|------|------|------|
| **绿常亮** | always_on | **PATROL 巡线** |
| **蓝闪 + 云台顶跑马** | flash + marquee | **SCAN** 扫人 / 换扫视大段 |
| **黄呼吸** | breath | 云台回中 / 扫到约 0° 段 |
| **红闪**（无枪口灯） | flash | 进 FIRE 告警 / 停火只瞄段 |
| **红闪 + 枪口灯亮** | flash + gun | **射击段**（应有脉冲开火） |
| **紫闪** | flash | 交战结束、即将整圈 SCAN |

发现后进 FIRE：应先告警再进入射击段（枪口灯开）；不因单帧无人而只红不射。

---

## 实现注意（API / 坑）

### 控制权

1. 进入 SCAN / FIRE 前：底盘 `stop`，`robot_mode_free`。  
2. PATROL：`robot_mode_chassis_follow` + 官方 RmList 循线。  
3. 离开 FIRE 前：`gun_ctrl.stop` 等停火。

### 阻塞与 sleep

- `yaw_ctrl` / `pitch_ctrl` / `angle_ctrl` / `rotate_with_degree`：执行块，**到位后返回**；勿再叠 sleep「等硬件」。  
- `rotate_with_speed`：速度环，需自行 `stop`。  
- `fire_once`：阻塞；本项目射击为脉冲 `fire_once`。

### 线 / 人

- 线：`err=cx-0.5`，`yaw=LINE_YAW_SIGN*err*Kp`（本机 SIGN=-1，见 PATROL 日志 cx 同号恶化）；固定速度；软区+`YAW_MAX`。  
- 人：API 报人先打 `PERSON raw`，再套几何带；过小/过大/aspect 打 `PERSON reject`。  
- 空地误检例：`wh≈(0.21,0.87) asp≈4.1` → `too_large`/`aspect`。

### 水弹

- 约 **俯仰 >10°** 机内禁止水晶弹；扫人抬头可能导致「有 BURST 日志但无弹」。

### 非目标

- 外部 PC SDK、Root  
- Mac 实时图传闭环  
- 黑线巡线  
