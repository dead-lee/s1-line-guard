# 设计备忘

## 权威行为规格

**状态机与已定业务规则以 [`behavior-spec.md`](./behavior-spec.md) 为准。**

改 `onboard/line_guard.py` 前必须对照该文档，避免引入与已定逻辑冲突的参数（例如历史上的空扫上限、脱战冷却、FIRE 总时长强制退出）。

---

## 实现注意（API / 坑）

### 控制权

1. 进入 SCAN / LOCK / FIRE 前：底盘 `stop`，`robot_mode_free`。  
2. PATROL：`robot_mode_chassis_follow` + 官方 RmList 循线。  
3. 离开 FIRE/LOCK 前：`gun_ctrl.stop` 等停火。

### 阻塞与 sleep

- `yaw_ctrl` / `pitch_ctrl` / `angle_ctrl` / `rotate_with_degree`：执行块，**到位后返回**；勿再叠 sleep「等硬件」。  
- `rotate_with_speed`：速度环，需自行 `stop`。  
- `fire_once`：阻塞；`fire_continuous`：非阻塞。

### 线 / 人

- 线：必须 `RmList(get_line_detection_info())`，`len==42` 且点数有效，`cx=[19]`。  
- 人：`enable_detection(people)`；轮询 `get_people_detection_info` 或官方 `cond_wait`（阻塞，不宜插在步进 SCAN 中途单独依赖）。  
- 行人误检（沙发/行李）官方无更好过滤器；几何启发式不可靠。

### 水弹

- 约 **俯仰 >10°** 机内禁止水晶弹；扫人抬头可能导致「有 BURST 日志但无弹」。

### 非目标

- 外部 PC SDK、Root  
- Mac 实时图传闭环  
- 黑线巡线  
