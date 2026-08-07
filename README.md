# S1 Line Guard（沿线哨兵）

基于 **DJI RoboMaster S1** 官方 App 实验室（车载 Python）的沿线巡逻卫兵程序。

- **Mac / 本仓库**：维护规格、车载单文件与调试资源  
- **S1 上**：粘贴 `onboard/line_guard.py` 后全自动运行  
- **不需要** Mac 与 S1 实时传数据；功能闭环全部在车上完成  

| 文档 | 用途 |
|------|------|
| **[`docs/behavior-spec.md`](docs/behavior-spec.md)** | **行为唯一权威**（改代码必读） |
| [`AGENTS.md`](AGENTS.md) | 助手纪律：规格未写禁止进代码；**文档须与代码一致** |
| [`docs/design-notes.md`](docs/design-notes.md) | 灯光含义、API 坑、实现备忘 |
| `onboard/line_guard.py` | **正式车载程序**（看文件首行 `LINE_GUARD_VERSION`） |

仓库：<https://github.com/dead-lee/s1-line-guard>

---

## 1. 做什么

S1 沿地面 **蓝色** 色带巡逻；定时停车扫视是否有人。发现行人后：

1. **hit≥3 即 FIRE**：告警 + 边瞄边射  
2. 节奏：**射约 3 s** → **停火只瞄约 3 s** → 再射 …  
3. 进入 FIRE 后：首段射击与告警按节奏执行；有检出用本帧框瞄准，无检出云台保持姿态；**首段射完后**连续 miss 超门槛才回 SCAN  
4. 无线：底盘停，**反复完整 SCAN**；有线再回 PATROL  

一句话：**沿线巡逻 → 停车扫人 → 发现即告警开火 → 做完再回扫/巡线。**

告警反馈为 **机内内置音效 + 灯色**（`line_guard.py` 的 `sfx` / `fx_*`）。

---

## 2. 状态机（与代码一致）

```
INIT
  │
  ▼
PATROL ──T_MOVE 贴线满──► SCAN
  ▲                         │
  │              hit≥3 发现人 │ 整圈无人
  │                         ▼
  │                        FIRE  ◄── 射 3s / 停瞄 3s
  │                         │
  │                    miss 放弃
  │                         ▼
  └──────── 有线 ◄──────── SCAN（可反复；无线继续 SCAN）
```

| 状态 | 行为 |
|------|------|
| **PATROL** | 低头循蓝线 `T_MOVE` 秒；**不认人** |
| **SCAN** | 停车；规划 yaw 步进扫视；步间查人 |
| **FIRE** | 告警 + 边瞄边射；SHOOT/HOLD 交替 |

细则见 [`docs/behavior-spec.md`](docs/behavior-spec.md)。

### 主要可调参数（`line_guard.py` CONFIG）

| 参数 | 当前约值 | 含义 |
|------|----------|------|
| `T_MOVE` | 6.0 s | 贴线多久进 SCAN |
| `PERSON_HIT_NEED` | 3 | 连续 hit≥3 才算发现 → FIRE |
| `PERSON_MISS_NEED` | 3 | 已射满 3s 后，连续 miss 超此 → SCAN |
| `T_FIRE_ON` / `T_FIRE_OFF` | 3.0 / 3.0 s | 射击段 / 停火只瞄段 |
| `SCAN_STEP_DEG` / `SCAN_LOOK_OPS` | 45° / 5 | 扫视步进；每角查人次数 |
| `PITCH_LINE` / `PITCH_SCAN` | −20 / 20 | 巡线低头 / 扫人抬头 |
| `LINE_SPEED` | 0.20 | 巡线固定前进速度 |
| `LINE_PID_KP/KI/KD` | 330 / 0 / 28 | 贴线 `PIDCtrl` 参数 |
| `PERSON_MIN/MAX_W` | 0.09～0.20 | 人体框宽度带 |
| `PERSON_MIN/MAX_H` | 0.50～0.85 | 人体框高度带 |
| `ENABLE_FIRE` | True | 是否允许水弹（联调可改 False） |

灯光对照见 [`docs/design-notes.md`](docs/design-notes.md)。

---

## 3. 技术边界

| 点 | 说明 |
|----|------|
| 单摄像头 | 巡线低头、扫人抬头 → **分时** |
| 入侵 | **检测到人**（w/h 尺寸带过滤；无精确测距） |
| 线颜色 | 仅红/绿/蓝；本项目固定 **蓝** |
| 车载 Python | 官方 API；**单文件**粘贴；无 pip / 不以 threading 为主 |
| 水弹 | 俯仰过高时机内可能禁射；安全区域自理 |

官方参考：[RoboMaster 开发者文档](https://robomaster-dev.readthedocs.io/zh-cn/latest/) · [S1 Programming Guide](https://www.dji.com/robomaster-s1/programming-guide)

---

## 4. 仓库结构

```text
s1-line-guard/
├── README.md
├── AGENTS.md
├── docs/
│   ├── behavior-spec.md   # 行为权威
│   ├── design-notes.md    # 灯光 / API 备忘
│   └── dev-plan.md        # 交付与联调
├── onboard/
│   ├── line_guard.py      # ★ 正式哨兵（粘贴此文件）
│   ├── README.md
│   ├── line_pitch_test.py / person_detect_test.py / led_color_test.py
│   └── wheel_clean.py     # 麦轮清洁
├── logs/                  # 本地调试截图（一般不入库）
├── resources/             # 预留
└── scripts/               # 预留
```

调试：App 控制台截图放入 `logs/`，对话说「看 logs」即可。

---

## 5. 场地与安全

- 浅色地面 + **蓝色**色带（约 15–25 mm）；推荐 **单环 / 跑道形**。  
- 首次联调可将 `ENABLE_FIRE = False`；注意弹道与急停。

### 上车步骤

1. 打开 `onboard/line_guard.py`，确认首行 **VERSION stamp**  
2. 全选复制 → App → 实验室 → Python → 粘贴运行  
3. 蓝线单环；观察 PATROL → SCAN →（有人）FIRE  

---

## 6. 许可

见 [LICENSE](./LICENSE)（MIT）。与 DJI / RoboMaster 商标及官方条款无关。

---

## 7. 联调与维护工具（`onboard/`）

正式交付是 `line_guard.py`。下列脚本在调通 baseline 时用过，**保留备用**；用法同样是全选粘贴进 App 实验室。

| 文件 | 用途 |
|------|------|
| `led_color_test.py` | 装甲/云台灯色与效果轮换，确认灯控 API |
| `line_pitch_test.py` | 蓝线识别、俯仰、RmList 字段（含 `cx`） |
| `person_detect_test.py` | 行人识别与灯效反馈 |
| `wheel_clean.py` | 麦轮慢速空转清洁（维护，非哨兵逻辑） |

建议顺序：灯 → 线 → 人 → 全流程 `line_guard.py`。详见 [`docs/dev-plan.md`](docs/dev-plan.md)、[`onboard/README.md`](onboard/README.md)。
