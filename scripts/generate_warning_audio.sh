#!/usr/bin/env bash
# 生成英文终结者/安保风格警告语音 → resources/warning_intruder.mp3
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
OUT_MP3="$ROOT/resources/warning_intruder.mp3"
OUT_ALT="$ROOT/resources/warning_intruder_alt_guy.mp3"
OUT_VTT="$ROOT/resources/warning_intruder.vtt"
TEXT="Intruder detected. Leave the area immediately. Or you will be fired upon. Consequences will be severe."

export PATH="${HOME}/Library/Python/3.9/bin:${PATH}"

if ! command -v edge-tts >/dev/null 2>&1; then
  echo "edge-tts not found. Install with: python3 -m pip install --user edge-tts"
  exit 1
fi

mkdir -p "$ROOT/resources"

# Primary: urgent, faster authoritative male
edge-tts \
  --voice "en-US-ChristopherNeural" \
  --rate="+25%" \
  --pitch="-10Hz" \
  --text "$TEXT" \
  --write-media "$OUT_MP3" \
  --write-subtitles "$OUT_VTT"

# Alternate: urgent Guy
edge-tts \
  --voice "en-US-GuyNeural" \
  --rate="+28%" \
  --pitch="-12Hz" \
  --text "$TEXT" \
  --write-media "$OUT_ALT"

echo "Wrote: $OUT_MP3"
echo "Wrote: $OUT_ALT"
ls -la "$OUT_MP3" "$OUT_ALT"
