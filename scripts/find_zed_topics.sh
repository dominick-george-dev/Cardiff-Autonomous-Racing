#!/usr/bin/env bash
# Find the real ZED camera topics on the car and diagnose why they're missing.
#
# Discovers image/depth/camera_info topics by MESSAGE TYPE, not by name, so it
# works whatever the wrapper version calls them (v4 vs v5 renamed things; v5
# also adds status/heartbeat topics that show up even when the camera itself
# failed to open). If no image topics exist it checks USB, the node, lifecycle
# state and the wrapper log to say why.
#
# Usage:
#   scripts/find_zed_topics.sh                   diagnose + print topic names
#   scripts/find_zed_topics.sh --launch [model]  also start the wrapper if down
#                                                (model: zed2 | zed2i, default zed2)
#   scripts/find_zed_topics.sh --env             print only export lines, so:
#       eval "$(scripts/find_zed_topics.sh --env)" && scripts/setup_real_car.sh

set -o pipefail

LAUNCH=0; ENV_ONLY=0; CAMERA_MODEL="${CAMERA_MODEL:-zed2}"
while [ $# -gt 0 ]; do
  case "$1" in
    --launch) LAUNCH=1; [ -n "${2:-}" ] && [[ "$2" != --* ]] && { CAMERA_MODEL="$2"; shift; } ;;
    --env)    ENV_ONLY=1 ;;
    *) echo "unknown arg: $1" >&2; exit 2 ;;
  esac
  shift
done

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
ZED_WS="${ZED_WS:-$HOME/zed_ws}"

# In --env mode keep stdout clean (export lines only); diagnostics go to stderr.
if [ "$ENV_ONLY" = 1 ]; then exec 3>&2; else exec 3>&1; fi
say()  { echo "$*" >&3; }
head_() { echo >&3; echo "== $* ==" >&3; }

[ -f /opt/ros/humble/setup.bash ] || { say "ERROR: ROS 2 Humble not installed"; exit 1; }
source /opt/ros/humble/setup.bash
[ -f "$ZED_WS/install/setup.bash" ] && source "$ZED_WS/install/setup.bash"

# ---------------------------------------------------------------------------
# Environment — a mismatch here between terminals makes topics "invisible"
# ---------------------------------------------------------------------------
head_ "ROS environment"
say "  ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-<unset (defaults to 0)>}"
say "  RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-<unset (Humble default: rmw_fastrtps_cpp)>}"
say "  (these must be IDENTICAL in the terminal running the camera and the one"
say "   running 'ros2 topic list' — setup_real_car.sh forces domain 0 + fastrtps)"

# Stale daemon caches are a classic source of wrong 'ros2 topic list' output.
ros2 daemon stop >/dev/null 2>&1; ros2 daemon start >/dev/null 2>&1; sleep 3

# ---------------------------------------------------------------------------
# Hardware + node
# ---------------------------------------------------------------------------
head_ "Camera hardware"
if lsusb 2>/dev/null | grep -qiE '2b03|stereolabs'; then
  say "  OK   ZED detected on USB: $(lsusb | grep -iE '2b03|stereolabs' | head -1)"
else
  say "  FAIL no ZED on USB — check the cable (must be USB3) before anything else"
fi

head_ "ZED wrapper"
if [ -d "$ZED_WS/src/zed-ros2-wrapper" ]; then
  VER="$(git -C "$ZED_WS/src/zed-ros2-wrapper" describe --tags 2>/dev/null || echo '?')"
  say "  wrapper source version: $VER"
fi
ZED_NODE="$(ros2 node list 2>/dev/null | grep -m1 -i zed)"
if [ -n "$ZED_NODE" ]; then
  say "  OK   node running: $ZED_NODE"
elif [ "$LAUNCH" = 1 ]; then
  [ -f "$ZED_WS/install/setup.bash" ] || { say "ERROR: $ZED_WS not built"; exit 1; }
  say "  launching zed_wrapper (camera_model:=$CAMERA_MODEL)..."
  mkdir -p "$REPO_ROOT/logs"
  nohup ros2 launch zed_wrapper zed_camera.launch.py camera_model:="$CAMERA_MODEL" \
    >"$REPO_ROOT/logs/zed_camera_doctor.log" 2>&1 &
  for i in $(seq 1 20); do
    sleep 3
    ros2 topic list -t 2>/dev/null | grep -q 'sensor_msgs/msg/Image' && break
    say "  waiting for image topics (${i}0s max 60s)... "
  done
  ZED_NODE="$(ros2 node list 2>/dev/null | grep -m1 -i zed)"
else
  say "  FAIL no zed node running — rerun with:  $0 --launch [zed2|zed2i]"
fi

# Newer wrappers can be lifecycle-managed: node exists but publishes nothing
# until activated.
LC_NODE="$(ros2 lifecycle nodes 2>/dev/null | grep -m1 -i zed)"
if [ -n "$LC_NODE" ]; then
  LC_STATE="$(ros2 lifecycle get "$LC_NODE" 2>/dev/null)"
  say "  lifecycle node: $LC_NODE  state: ${LC_STATE:-?}"
  if [ -n "$LC_STATE" ] && ! echo "$LC_STATE" | grep -qi active; then
    if [ "$LAUNCH" = 1 ]; then
      say "  activating..."
      ros2 lifecycle set "$LC_NODE" configure >/dev/null 2>&1
      ros2 lifecycle set "$LC_NODE" activate  >/dev/null 2>&1 && sleep 3
    else
      say "  -> not active! run: ros2 lifecycle set $LC_NODE configure && ros2 lifecycle set $LC_NODE activate"
    fi
  fi
fi

# ---------------------------------------------------------------------------
# Topic discovery by message type
# ---------------------------------------------------------------------------
head_ "Topics on the graph (by type)"
TOPICS="$(ros2 topic list -t 2>/dev/null)"

by_type() { echo "$TOPICS" | grep -F "[$1]" | awk '{print $1}'; }
IMAGES="$(by_type sensor_msgs/msg/Image)"
INFOS="$(by_type sensor_msgs/msg/CameraInfo)"
IMUS="$(by_type sensor_msgs/msg/Imu)"

say "  Image topics:"
say "$(echo "${IMAGES:-  (none)}" | sed 's/^/    /')"
say "  CameraInfo topics:"
say "$(echo "${INFOS:-  (none)}" | sed 's/^/    /')"
say "  IMU topics:"
say "$(echo "${IMUS:-  (none)}" | sed 's/^/    /')"

pick() {  # pick <candidates> <regex...> — first candidate matching earliest regex
  local cands="$1"; shift
  for re in "$@"; do
    local hit; hit="$(echo "$cands" | grep -E "$re" | head -1)"
    [ -n "$hit" ] && { echo "$hit"; return; }
  done
}

RGB_CANDS="$(echo "$IMAGES" | grep -viE 'depth|confidence|disparity|gray|raw_')"
RGB="$(pick "$RGB_CANDS" 'rgb/image_rect_color$' 'left/image_rect_color$' 'rgb.*color' 'left.*color' '.')"
DEPTH="$(pick "$(echo "$IMAGES" | grep -iE 'depth')" 'depth_registered$' '.')"
INFO=""
if [ -n "$RGB" ]; then INFO="$(echo "$INFOS" | grep -F "${RGB%/*}/camera_info" | head -1)"; fi
[ -z "$INFO" ] && INFO="$(pick "$(echo "$INFOS" | grep -viE 'depth')" 'left/camera_info$' 'rgb/camera_info$' '.')"

# ---------------------------------------------------------------------------
# No image topics at all -> dig into why
# ---------------------------------------------------------------------------
if [ -z "$RGB" ]; then
  head_ "No image topics found — wrapper log"
  ZLOG="$(ls -t "$REPO_ROOT"/logs/*/zed*.log "$REPO_ROOT"/logs/zed*.log 2>/dev/null | head -1)"
  [ -z "$ZLOG" ] && ZLOG="$(ls -t "$HOME"/.ros/log/*/launch.log 2>/dev/null | head -1)"
  if [ -n "$ZLOG" ]; then
    say "  $ZLOG (errors/warnings):"
    grep -iE 'error|fatal|fail|not detected|no zed|bandwidth|usb|cuda|calibrat' "$ZLOG" \
      | tail -15 | sed 's/^/    /' >&3
  else
    say "  no zed log found under $REPO_ROOT/logs or ~/.ros/log"
  fi
  say ""
  say "  Likely causes, in order:"
  say "   1. camera not opened: bad/USB2 cable, or another process holds it"
  say "      (ZED allows ONE consumer — close ZED_Explorer / other wrapper instances)"
  say "   2. SDK/wrapper mismatch: SDK 5.1.2 needs wrapper humble-v5.x; SDK 4.2.5"
  say "      needs humble-v4.2.5 (see ON_CAR_SETUP.md step 4) — rebuild zed_ws after switching"
  say "   3. lifecycle node not activated (see above)"
  say "   4. ROS_DOMAIN_ID / RMW_IMPLEMENTATION differ between terminals (see top)"
  exit 1
fi

# ---------------------------------------------------------------------------
# Verify frames are actually flowing (topics can be advertised but silent).
# Subscribing here also wakes the wrapper's publish-on-demand mode.
# ---------------------------------------------------------------------------
head_ "Frame rate check ($RGB)"
HZ="$(timeout 12 ros2 topic hz "$RGB" --window 20 2>/dev/null | grep -m1 'average rate')"
if [ -n "$HZ" ]; then
  say "  OK   $HZ"
else
  say "  WARN topic advertised but NO frames in 12s — camera grab is failing;"
  say "       check the wrapper log and USB bandwidth (must be a USB3 port/cable)"
fi

# ---------------------------------------------------------------------------
# Result
# ---------------------------------------------------------------------------
head_ "Detected camera topics"
say "  RGB_TOPIC=$RGB"
say "  DEPTH_TOPIC=${DEPTH:-<none found!>}"
say "  CAMERA_INFO_TOPIC=${INFO:-<none found!>}"

DEFAULT_RGB="/zed/zed_node/rgb/image_rect_color"
if [ "$RGB" != "$DEFAULT_RGB" ]; then
  say ""
  say "  NOTE: names differ from the defaults baked into setup_real_car.sh /"
  say "  launch_skidpad_car.sh — launch perception with the overrides below."
fi
say ""
say "  Bring up perception with these names:"
say "    RGB_TOPIC=$RGB DEPTH_TOPIC=${DEPTH:-$RGB} CAMERA_INFO_TOPIC=${INFO:-?} \\"
say "      scripts/setup_real_car.sh"
say ""
say "  Or run the detector directly:"
say "    ros2 run cone_detector YOLO_cone_detector --ros-args \\"
say "      -p rgb_topic:=$RGB \\"
say "      -p depth_topic:=${DEPTH:-$RGB} \\"
say "      -p camera_info_topic:=${INFO:-?}"

if [ "$ENV_ONLY" = 1 ]; then
  echo "export RGB_TOPIC=$RGB"
  [ -n "$DEPTH" ] && echo "export DEPTH_TOPIC=$DEPTH"
  [ -n "$INFO" ]  && echo "export CAMERA_INFO_TOPIC=$INFO"
fi
