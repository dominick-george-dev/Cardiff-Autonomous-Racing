#!/usr/bin/env bash
# TRACKDRIVE on the REAL CAR (rules D8): one command runs the whole stack.
#
# Thin wrapper over run_full_autonomous_lap_car.sh with the D8 requirements
# baked in:
#   - 10 laps, counted autonomously by the follower (D8.2.8 — no last-lap
#     signal is sent to the vehicle)
#   - the follower only moves when the AMI selector shows TRACKDRIVE and
#     ros_can reports AS:DRIVING (RES 'Go' starts the run, D8.2.6)
#   - after lap 10 the car keeps driving until the finish line is behind it
#     (staged 6 m before the line, D8.2.5) and then brakes — full stop well
#     inside the 30 m allowance, then mission_completed -> AS Finished
#     (D8.2.7)
#
# Staging: front wheels 6 m before the start line (D8.2.5), pointed down the
# track, perception already seeing cones. If the SLAM reference point is far
# from the nose, adjust FINISH_OVERRUN_M accordingly (it is measured from
# where the car SAT at the go signal; 8.0 = ~2 m past the line + margin).
#
# Usage:  scripts/run_trackdrive_car.sh [speed_mps]     (default 2.5)
#         scripts/run_trackdrive_car.sh stop
#
# First run of the day: do a 1-lap shakedown first:
#   scripts/run_full_autonomous_lap_car.sh 1 2.0

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [ "${1:-}" = "stop" ]; then
  exec "${SCRIPT_DIR}/run_full_autonomous_lap_car.sh" stop
fi

SPEED="${1:-2.5}"
LAPS=10
FINISH_OVERRUN_M=8.0

export FOLLOWER_EXTRA_ARGS="--require-ami TRACKDRIVE --finish-overrun ${FINISH_OVERRUN_M}"
exec "${SCRIPT_DIR}/run_full_autonomous_lap_car.sh" "${LAPS}" "${SPEED}"
