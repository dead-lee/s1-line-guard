# resources

| 文件 | 说明 |
|---|---|
| `warning_intruder.mp3` | **最终警告音（英文、急促）** |
| `warning_intruder.vtt` | 字幕时间轴 |

文案：

```
Intruder detected. Leave the area immediately. Or you will be fired upon. Consequences will be severe.
```

中文语义：发现入侵，请立刻离开，否则开火，后果自负。

参数：`en-US-ChristopherNeural`，rate `+25%`，pitch `-10Hz`。

重生成：`../scripts/generate_warning_audio.sh`

S1 实验室未必能直接播放自定义 MP3；实现时默认内置音效 + 红灯，本文件作素材与验收对照。
