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
| **红闪**（无枪口灯） | flash | **交战中未射**：进 FIRE / IR / 停火只瞄（3s） |
| **红闪 + 枪口灯亮** | flash + gun | **FIRE 射击段**（约 3s 在射水弹） |
| **紫闪** | flash | **确认丢人**，即将整圈 SCAN 重扫 |

易混两点：

1. **紫闪 = 丢人**；红闪 = 有人交战。  
2. **枪口灯开** = 正在射；同为红闪但无枪口 = 报警/瞄准/停火段。

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

- 线：必须 `RmList(get_line_detection_info())`，`len>=42` 且点数有效；控制用近/中/远融合 cx（远点约 `[19]`）。  
- 人：`enable_detection(people)`；轮询 `get_people_detection_info` 或官方 `cond_wait`（阻塞，不宜插在步进 SCAN 中途单独依赖）。  
- 行人误检（沙发/行李）官方无更好过滤器；几何启发式不可靠。

### 水弹

- 约 **俯仰 >10°** 机内禁止水晶弹；扫人抬头可能导致「有 BURST 日志但无弹」。

### 非目标

- 外部 PC SDK、Root  
- Mac 实时图传闭环  
- 黑线巡线  
