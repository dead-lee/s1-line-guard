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

1. **立即 FIRE**：边瞄准边射击  
2. 节奏：**射约 3 s** → **停火只瞄约 3 s** → 再射 …  
3. 首段 3 s 射击完成前不因 miss 放弃；之后连续 miss 超过门槛 → 停火，整圈 SCAN  
4. 无线：底盘停，**反复完整 SCAN**；有线再回 PATROL  

一句话：**沿线巡逻 → 停车扫人 → 发现即交战 → 按规格回扫或巡线。**

自定义警告 mp3（`resources/`）为素材；车载默认用 **内置音效 + 灯色**（App 未必能播自定义文件）。

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
| **FIRE** | 立即边瞄边射；SHOOT/HOLD 交替 |

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
| `LINE_SPEED` | 0.12 | 巡线固定前进速度 |
| `ENABLE_FIRE` | True | 是否允许水弹（联调可改 False） |

灯光对照见 [`docs/design-notes.md`](docs/design-notes.md)。

---

## 3. 技术边界

| 点 | 说明 |
|----|------|
| 单摄像头 | 巡线低头、扫人抬头 → **分时** |
| 入侵 | **检测到人**（几何过滤减误报；无精确测距） |
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
│   └── dev-plan.md        # 里程碑摘要（须与现状一致）
├── onboard/
│   ├── line_guard.py      # ★ 正式哨兵（粘贴此文件）
│   ├── README.md
│   ├── line_pitch_test.py / person_detect_test.py / led_color_test.py
│   └── wheel_clean.py     # 麦轮清洁（独立工具）
├── resources/             # 警告音素材（非车载必需）
├── logs/                  # 本地调试截图（一般不入库）
└── scripts/
```

调试：App 控制台截图放入 `logs/`，对话说「看 logs」即可。

---

## 5. 场地与安全

- 浅色地面 + **蓝色**色带（约 15–25 mm）；推荐 **单环 / 跑道形**（默认不做 8 字路口策略）。  
- 8 字交叉口易半环绕圈或丢线，不推荐作默认场地。  
- 首次联调可将 `ENABLE_FIRE = False`；注意弹道与急停。

### 上车步骤

1. 打开 `onboard/line_guard.py`，确认首行 **VERSION stamp**  
2. 全选复制 → App → 实验室 → Python → 粘贴运行  
3. 蓝线单环；观察 PATROL → SCAN →（有人）FIRE  

---

## 6. 许可

见 [LICENSE](./LICENSE)（MIT）。与 DJI / RoboMaster 商标及官方条款无关。
