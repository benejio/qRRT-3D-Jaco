#!/usr/bin/env bash
set -eo pipefail

cd ~/ws_3dof
source /opt/ros/humble/setup.bash
source install/setup.bash

mkdir -p ~/ws_3dof/results/qrrt_candidate_sweep_40

COMMON_ARGS="\
  --bins-per-joint 10 \
  --max-iters 2000 \
  --quantum-iters 1 \
  --quantum-shots 128 \
  --quantum-best-rounds 3 \
  --quantum-target-weight 1.0 \
  --quantum-goal-weight 1.0 \
  --quantum-score-margin 0.0"

CANDIDATES=(64 128 256 512)
SEEDS=$(seq 1 40)

seed_done () {
  local file="$1"
  local seed="$2"

  [ -f "$file" ] || return 1

  python3 - "$file" "$seed" <<'PY'
import sys
import pandas as pd

path = sys.argv[1]
seed = int(sys.argv[2])

try:
    df = pd.read_csv(path)
except Exception:
    sys.exit(1)

if "seed" not in df.columns:
    sys.exit(1)

done = set(df["seed"].dropna().astype(int))
sys.exit(0 if seed in done else 1)
PY
}

add_scene3_obstacles () {
  ros2 run qrrt_planner add_box_obstacle --remove --id box1 || true
  ros2 run qrrt_planner add_box_obstacle --remove --id cylinder1 || true
  ros2 run qrrt_planner add_box_obstacle --remove --id sphere1 || true

  ros2 run qrrt_planner add_box_obstacle \
    --id box1 \
    --shape box \
    --x 0.25 --y 0.00 --z 0.38 \
    --sx 0.30 --sy 0.95 --sz 0.10

  ros2 run qrrt_planner add_box_obstacle \
    --id cylinder1 \
    --shape cylinder \
    --x 0.25 --y 0.22 --z 0.58 \
    --sx 0.07 --sz 0.30

  ros2 run qrrt_planner add_box_obstacle \
    --id sphere1 \
    --shape sphere \
    --x 0.25 --y -0.22 --z 0.52 \
    --sx 0.09

  sleep 1
}

add_scene4_obstacles () {
  ros2 run qrrt_planner add_box_obstacle --remove --id box1 || true
  ros2 run qrrt_planner add_box_obstacle --remove --id cylinder1 || true
  ros2 run qrrt_planner add_box_obstacle --remove --id sphere1 || true

  ros2 run qrrt_planner add_box_obstacle \
    --id box1 \
    --shape box \
    --x 0.25 --y 0.00 --z 0.38 \
    --sx 0.30 --sy 0.95 --sz 0.10

  ros2 run qrrt_planner add_box_obstacle \
    --id cylinder1 \
    --shape cylinder \
    --x 0.25 --y 0.00 --z 0.58 \
    --sx 0.07 --sz 0.30

  ros2 run qrrt_planner add_box_obstacle \
    --id sphere1 \
    --shape sphere \
    --x 0.25 --y 0.25 --z 0.52 \
    --sx 0.09

  sleep 1
}

run_scene3 () {
  add_scene3_obstacles

  for kc in "${CANDIDATES[@]}"; do
    OUT=~/ws_3dof/results/qrrt_candidate_sweep_40/qrrt_scene3_kc${kc}_40seed.csv

    for seed in $SEEDS; do
      if seed_done "$OUT" "$seed"; then
        echo "qRRT Scene 3 | kc=${kc} | seed=${seed} already complete, skipping"
        continue
      fi

      echo "qRRT Scene 3 | kc=${kc} | seed=${seed}"

      ros2 run qrrt_planner run_qrrt_grid \
        $COMMON_ARGS \
        --quantum-candidates "$kc" \
        --q-start="0.0,3.142,3.142,0.0,0.0,0.0" \
        --q-goal="-1.431,4.189,5.009,2.251,-2.461,-1.536" \
        --seed "$seed" \
        --csv-out "$OUT"
    done
  done
}

run_scene4 () {
  add_scene4_obstacles

  for kc in "${CANDIDATES[@]}"; do
    OUT=~/ws_3dof/results/qrrt_candidate_sweep_40/qrrt_scene4_kc${kc}_40seed.csv

    for seed in $SEEDS; do
      if seed_done "$OUT" "$seed"; then
        echo "qRRT Scene 4 | kc=${kc} | seed=${seed} already complete, skipping"
        continue
      fi

      echo "qRRT Scene 4 | kc=${kc} | seed=${seed}"

      ros2 run qrrt_planner run_qrrt_grid \
        $COMMON_ARGS \
        --quantum-candidates "$kc" \
        --q-start="-2.094,3.194,1.693,-1.274,1.117,2.356" \
        --q-goal="-1.658,3.142,4.660,2.042,-2.374,-2.356" \
        --seed "$seed" \
        --csv-out "$OUT"
    done
  done
}

run_scene3
run_scene4

echo "Done."
echo "qRRT results saved in ~/ws_3dof/results/qrrt_candidate_sweep_40"
