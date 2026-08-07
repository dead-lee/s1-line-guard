# 联调与交付

**行为以 [`behavior-spec.md`](./behavior-spec.md) 为准；实现以 `onboard/line_guard.py` 首行 `LINE_GUARD_VERSION` 为准。**

---

## 交付内容

| 能力 | 说明 |
|------|------|
| 单文件车载 | `onboard/line_guard.py` 粘贴进 App 实验室 |
| 状态 | **PATROL ⇄ SCAN → FIRE** |
| 巡线 | 蓝线 + `chassis_follow`；`[19]` + 纯 P yaw + 固定速度 0.20 |
| 扫描 | 规划角步进约 45°；每角最多 5 次查人；hit≥3 → 立即 FIRE |
| 交战 | 发现即告警+开火；射 3s / 停瞄 3s；丢人不停射；首段射完后 miss 才可放弃 |
| 无线 | 反复完整 SCAN，有线再 PATROL |
| 灯光 | 见 `design-notes.md` |

---

## 联调顺序

1. **灯 / API**：`led_color_test.py`  
2. **线**：`line_pitch_test.py`（确认 RmList、俯仰可见线）  
3. **人**：`person_detect_test.py`  
4. **全流程**：`line_guard.py`  
   - PATROL 贴线满 `T_MOVE` → SCAN  
   - 无人整圈 → 有线回 PATROL / 无线再 SCAN  
   - 有人 hit≥3 → FIRE（射/停交替）  
5. 先 `ENABLE_FIRE = False` 再开水弹  

---

## 范围外（除非改 behavior-spec）

- 自定义 mp3 依赖、Mac 实时图传闭环  
- 黑线巡线、PC SDK  
- 规格未写的冷却 / 空扫上限 / coast 等  
