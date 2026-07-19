#!/usr/bin/env bash
# Shared ZED auto-detection helpers. Source this file, then call:
#
#   detect_camera_model   sets CAMERA_MODEL from lsusb (honours a pre-set
#                         value other than "auto"/empty)
#   any_zed_image_up      returns 0 if any ZED image topic is live
#   resolve_zed_topics    sets RGB_TOPIC / DEPTH_TOPIC / CAMERA_INFO_TOPIC
#                         from the live topic list (honours pre-set values),
#                         verifies message types, prints what it picked;
#                         returns 1 if a required topic cannot be found
#
# Used by setup_real_car.sh and launch_skidpad_car.sh so the car never
# depends on hardcoded wrapper topic names / camera model.

detect_camera_model() {
  if [ -n "${CAMERA_MODEL:-}" ] && [ "$CAMERA_MODEL" != "auto" ]; then
    echo "  camera model (preset): $CAMERA_MODEL"
    return 0
  fi
  local usb
  usb="$(lsusb 2>/dev/null | grep -iE 'zed|stereolabs' | head -n1)"
  case "$usb" in
    *[Zz][Ee][Dd]*2i*)        CAMERA_MODEL="zed2i" ;;
    *[Zz][Ee][Dd]*2*)         CAMERA_MODEL="zed2"  ;;
    *[Zz][Ee][Dd]*[Xx]*[Mm]*) CAMERA_MODEL="zedxm" ;;
    *[Zz][Ee][Dd]*[Xx]*)      CAMERA_MODEL="zedx"  ;;
    *[Mm]ini*)                CAMERA_MODEL="zedm"  ;;
    *)                        CAMERA_MODEL="zed2"  ;;
  esac
  if [ -n "$usb" ]; then
    echo "  camera model from USB: $CAMERA_MODEL  ($usb)"
  else
    echo "  WARN: no ZED visible on USB (lsusb) — defaulting camera_model=$CAMERA_MODEL"
  fi
}

any_zed_image_up() {
  ros2 topic list 2>/dev/null | grep -E 'zed[^ ]*image' | grep -q .
}

resolve_zed_topics() {
  local topics
  topics="$(ros2 topic list 2>/dev/null)"

  _pick_topic() {  # first live match wins; skips right-eye/gray/confidence
    local pat t
    for pat in "$@"; do
      t="$(echo "$topics" | grep -E "$pat" | grep -vE '/right/|gray|confidence' | head -n1)"
      if [ -n "$t" ]; then echo "$t"; return 0; fi
    done
    return 1
  }

  if [ -z "${RGB_TOPIC:-}" ]; then
    RGB_TOPIC="$(_pick_topic '/rgb/image_rect_color$' '/left/image_rect_color$' \
                             'image_rect_color$' '/rgb/image_raw$' '/left/image_raw$')" || {
      echo "  ERROR: no ZED RGB image topic found — zed topics: $(echo "$topics" | grep -i zed | tr '\n' ' ')"
      return 1
    }
  fi
  if [ -z "${DEPTH_TOPIC:-}" ]; then
    DEPTH_TOPIC="$(_pick_topic '/depth/depth_registered$' '/depth/image$' '/depth/image_raw$')" || {
      echo "  ERROR: no ZED depth topic found — zed topics: $(echo "$topics" | grep -i zed | tr '\n' ' ')"
      return 1
    }
  fi
  if [ -z "${CAMERA_INFO_TOPIC:-}" ]; then
    # prefer the camera_info that sits next to the chosen RGB topic
    local sibling="${RGB_TOPIC%/*}/camera_info"
    if echo "$topics" | grep -qx "$sibling"; then
      CAMERA_INFO_TOPIC="$sibling"
    else
      CAMERA_INFO_TOPIC="$(_pick_topic '/left/camera_info$' '/rgb/camera_info$' 'camera_info$')" || {
        echo "  ERROR: no camera_info topic found — zed topics: $(echo "$topics" | grep -i zed | tr '\n' ' ')"
        return 1
      }
    fi
  fi

  echo "  RGB:         $RGB_TOPIC"
  echo "  DEPTH:       $DEPTH_TOPIC"
  echo "  CAMERA_INFO: $CAMERA_INFO_TOPIC"

  # sanity-check message types (warn only — an odd type usually means the
  # pattern grabbed the wrong topic; override via env vars if so)
  local pair tp want got
  for pair in "$RGB_TOPIC|sensor_msgs/msg/Image" \
              "$DEPTH_TOPIC|sensor_msgs/msg/Image" \
              "$CAMERA_INFO_TOPIC|sensor_msgs/msg/CameraInfo"; do
    tp="${pair%%|*}"; want="${pair##*|}"
    got="$(timeout 5 ros2 topic type "$tp" 2>/dev/null | head -n1)"
    [ "$got" = "$want" ] || echo "  WARN: $tp has type '${got:-unknown}' (expected $want)"
  done
  return 0
}
