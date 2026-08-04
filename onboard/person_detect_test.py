# PERSON_DETECT_TEST_VERSION=1.1.0 stamp=2026-08-04 14:30:00
# -*- coding: utf-8 -*-
#
# S1 行人识别测试 — 严格按官方示例结构
#
# 官方下载示例（你提供）：
#   def start():
#       vision_ctrl.enable_detection(rm_define.vision_detection_people)
#       while True:
#           vision_ctrl.cond_wait(rm_define.cond_recognized_people)
#           media_ctrl.play_sound(rm_define.media_custom_audio_0)
#           led_ctrl.set_bottom_led(..., effect_flash)
#           led_ctrl.set_top_led(..., effect_marquee)
#           time.sleep(15)
#
# 官方要点：
#   1) 只 enable people
#   2) 用 cond_wait(cond_recognized_people) 阻塞等待「识别到人」——不是轮询 get_people_detection_info
#   3) 等到之后再播音 + 改灯
#   4) sleep 一段时间再进入下一轮 wait
#
# 本文件在官方结构上仅增加：版本打印、free/云台姿态、自定义音失败时的兜底音效。
# 无巡线、无射击、无自写解析。

HOLD_AFTER_DETECT_S = 15

def start():
    print("======== Person Detect Test (official cond_wait) ========")
    print("# PERSON_DETECT_TEST_VERSION=1.1.0 stamp=2026-08-04 14:30:00")
    print("[PDT] path = enable_detection(people) + cond_wait(cond_recognized_people)")

    # 官方示例未设 mode；为稳定云台加 free + 略抬头（可选，不改识别 API）
    try:
        robot_ctrl.set_mode(rm_define.robot_mode_free)
        chassis_ctrl.stop()
        gimbal_ctrl.yaw_ctrl(0)
        gimbal_ctrl.pitch_ctrl(10)
    except Exception:
        pass

    # ----- 以下与官方一致 -----
    vision_ctrl.enable_detection(rm_define.vision_detection_people)

    while True:
        # 阻塞：直到系统判定「识别到人」
        print("[PDT] waiting cond_recognized_people ...")
        vision_ctrl.cond_wait(rm_define.cond_recognized_people)
        print("[PDT] RECOGNIZED people")

        # 官方：自定义音 0；若工程无自定义音则兜底内置音
        try:
            media_ctrl.play_sound(rm_define.media_custom_audio_0)
        except Exception:
            try:
                media_ctrl.play_sound(rm_define.media_sound_recognize_success)
            except Exception:
                pass

        # 官方灯效
        led_ctrl.set_bottom_led(
            rm_define.armor_bottom_all, 0, 127, 70, rm_define.effect_flash
        )
        led_ctrl.set_top_led(
            rm_define.armor_top_all, 69, 215, 255, rm_define.effect_marquee
        )
        try:
            led_ctrl.set_flash(rm_define.armor_all, 3)
        except Exception:
            pass

        time.sleep(HOLD_AFTER_DETECT_S)
