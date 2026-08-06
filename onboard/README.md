# onboard — 车载代码（App 实验室）

此目录 Python 用于粘贴到 **RoboMaster App → 实验室 → Python**。

## 平台限制

- **单文件交付**：整份逻辑在一个 `.py` 里全选粘贴。  
- **不要** `import` 本仓库其它本地模块。  
- 仅用官方车载 API；禁止 `numpy` / `cv2` / pip / 以 `threading` 为主的并发。

## 文件

| 文件 | 说明 |
|------|------|
| **`line_guard.py`** | **正式哨兵**（粘贴运行；看首行 `LINE_GUARD_VERSION`） |
| `line_pitch_test.py` | 蓝线识别 / 俯仰 / RmList dump |
| `person_detect_test.py` | 行人识别灯效测试 |
| `led_color_test.py` | 彩灯测试 |
| `wheel_clean.py` | 麦轮清洁（慢速转轮） |
| `main.py` | 仅说明占位，**不可运行** |
| `config.py` | **废弃**，参数以 `line_guard.py` CONFIG 为准 |

行为权威：[`../docs/behavior-spec.md`](../docs/behavior-spec.md)。

## 运行 `line_guard.py`

1. 确认首行 VERSION stamp 为仓库最新  
2. 全选复制 → App → 实验室 → Python → 粘贴运行  
3. 蓝胶带单环；状态：**PATROL → SCAN →（hit≥3）FIRE**  
4. 联调可先 `ENABLE_FIRE = False`  
5. 反馈时附：VERSION、状态日志、报错原文、要调的参数  

## 运行测试脚本

任选一个测试文件，同样「全选粘贴」运行。`led_color_test` 预期：底盘+云台灯色轮换后熄灭。

## 安全

- 测灯 / 测线可不装弹；开火测试注意弹道与急停。  
- 水弹在俯仰过高时可能被机内禁止。  
