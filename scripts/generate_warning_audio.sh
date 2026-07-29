#!/usr/bin/env bash
# 重新生成终结者风格警告语音 → resources/warning_intruder.mp3
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
OUT_MP3="$ROOT/resources/warning_intruder.mp3"
OUT_VTT="$ROOT/resources/warning_intruder.vtt"
TEXT="发现入侵，请立刻离开，否则开火，后果自负。"

export PATH="${HOME}/Library/Python/3.9/bin:${PATH}"

if ! command -v edge-tts >/dev/null 2>&1; then
  echo "edge-tts not found. Install with: python3 -m pip install --user edge-tts"
  exit 1
fi

mkdir -p "$ROOT/resources"
# 低沉、偏慢、机械感（可按口味改 rate/pitch/voice）
edge-tts \
  --voice "zh-CN-YunjianNeural" \
  --rate="-20%" \
  --pitch="-25Hz" \
  --text "$TEXT" \
  --write-media "$OUT_MP3" \
  --write-subtitles "$OUT_VTT"

echo "Wrote: $OUT_MP3"
ls -la "$OUT_MP3"
