#!/bin/bash
# Smoke-test the Hatari RRDC backend (contract 0.5.0): bind + marker, then
# exercise /status, /regs, /screenshot, and deterministic /pause + /step.
#
# Runs headless via SDL's offscreen driver. --alert-level fatal is REQUIRED so a
# TOS/machine-mismatch alert is logged instead of popping a modal dialog that
# would block forever with no display.
#
# Usage: ./rrdc-smoke.sh [/path/to/tos.img]   (default: tos/TOS162UK.ROM)
set -u
HAT=/home/andymccall/development/hatari/build-rrdc/src/hatari
TOS="${1:-/home/andymccall/development/hatari/tos/TOS162UK.ROM}"
PORT=$(( (RANDOM % 20000) + 40000 ))
LOG=$(mktemp)
fail=0

[ -x "$HAT" ] || { echo "build first: cmake --build build-rrdc"; exit 1; }
[ -r "$TOS" ] || { echo "TOS image not readable: $TOS"; exit 1; }

echo "== hatari-rrdc headless, port $PORT, TOS $(basename "$TOS") =="
SDL_VIDEODRIVER=offscreen SDL_AUDIODRIVER=dummy HATARI_CONTROLPORT=$PORT \
  "$HAT" --tos "$TOS" --alert-level fatal --fast-forward on --sound off >"$LOG" 2>&1 &
HPID=$!
trap 'kill $HPID 2>/dev/null; rm -f "$LOG"' EXIT

for i in $(seq 1 50); do
  grep -q '\[retro_control\] listening' "$LOG" && break
  kill -0 "$HPID" 2>/dev/null || { echo "FAIL: hatari exited early"; tail -6 "$LOG"; exit 1; }
  sleep 0.2
done
grep -q '\[retro_control\] listening' "$LOG" || { echo "FAIL: no bind marker"; tail -6 "$LOG"; exit 1; }
echo "OK  marker: $(grep retro_control "$LOG")"

st () { curl -s --max-time 3 "http://127.0.0.1:$PORT/$1"; }
stp () { curl -s --max-time 3 -X POST "http://127.0.0.1:$PORT/$1" >/dev/null; }
jq_get () { python3 -c "import sys,json;print(json.load(sys.stdin)['$1'])"; }

st status | grep -q '"contract":"0.5.0"' && echo "OK  /status 0.5.0" || { echo "FAIL /status"; fail=1; }
st regs   | grep -q '"pc"'               && echo "OK  /regs"        || { echo "FAIL /regs"; fail=1; }

curl -s --max-time 5 "http://127.0.0.1:$PORT/screenshot" -o /tmp/hatari_shot.ppm
if [ "$(head -c2 /tmp/hatari_shot.ppm)" = "P6" ]; then
  echo "OK  /screenshot $(head -1 /tmp/hatari_shot.ppm | tr -d 'P6') ($(wc -c </tmp/hatari_shot.ppm) bytes)"
else echo "FAIL /screenshot"; fail=1; fi

stp pause
f1=$(st status | jq_get frame)
stp "step?frames=10"
sleep 0.4
f2=$(st status | jq_get frame)
if [ "$f2" = "$((f1 + 10))" ]; then echo "OK  /step: $f1 -> $f2 (+10)"; else echo "FAIL /step: $f1 -> $f2"; fail=1; fi

[ "$fail" = 0 ] && echo "== ALL PASS ==" || echo "== FAILURES =="
exit $fail
