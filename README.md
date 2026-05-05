# QRRT Planner on Discretized Joint-Space Grids

This repository contains a ROS 2 / MoveIt-based motion-planning testbed for a 6-DOF Kinova JACO arm. The current focus is comparing:

- **Classical RRT on a discretized joint-space grid**
- **Quantum-guided RRT on a discretized joint-space grid**
- **Grid-based A\*** on the same discretized configuration space

The planner operates in a **discretized 6D joint configuration space** while collision checking is still performed against the continuous robot and obstacle geometry through MoveIt.

---

## Current status

Working components:

- Discretized joint-space grid generation
- MoveIt collision checking through `/check_state_validity`
- Classical grid-RRT
- Quantum grid-RRT with Grover-based local candidate selection
- Grid A\* baseline
- Planning scene obstacle insertion for box / sphere / cylinder
- CSV and debug logging support in the runner scripts

Current best-tested quantum settings vary by scenario:

- Scenario 1: `quantum_candidates = 512`
- Scenario 2: `quantum_candidates = 256`
- Scenario 3: `quantum_candidates = 512`

Common quantum settings used in the candidate sweeps:

- `quantum_top_k = 8`
- `quantum_iters = 1`
- `quantum_shots = 128`

The quantum planner is functioning. Classical grid-RRT is generally faster and more stable overall, but Scenario 3 showed a case where quantum-guided grid-RRT outperformed classical grid-RRT at `k = 512`.

---

## Repository structure

Main Python files under `qrrt_planner/`:

- `moveit_collision_checker.py`  
  Connects to MoveIt and queries state validity.

- `grid_joint_space.py`  
  Joint-space discretization utilities and grid helpers.

- `run_grid_astar.py`  
  Runs grid-based A\* on the discretized joint space.

- `benchmark_grid_astar.py`  
  Repeated grid-A\* benchmark runner with CSV output.

- `crrt_grid.py`  
  Classical RRT over the discretized joint-space grid.

- `run_crrt_grid.py`  
  Command-line runner for the classical grid-RRT.

- `grid_quantum_sampler.py`  
  Quantum candidate scoring / Grover-based local selection.

- `qrrt_grid.py`  
  Quantum RRT over the discretized joint-space grid.

- `run_qrrt_grid.py`  
  Command-line runner for the quantum grid-RRT.

- `oracle_qiskit.py`  
  Grover / oracle utilities used by the quantum sampler.

- `add_box_obstacle.py`  
  Adds or removes obstacles in the MoveIt planning scene.

---

## System requirements

Tested environment:

- Ubuntu 22.04
- ROS 2 Humble
- MoveIt 2
- Python 3.10
- Kinova JACO MoveIt configuration
- Qiskit

---

## Build

From the workspace root:

```bash
source /opt/ros/humble/setup.bash
cd ~/ws_3dof
colcon build --symlink-install --packages-select qrrt_planner
source ~/ws_3dof/install/setup.bash
```

---

## Launch MoveIt

Before running any planner, `move_group` must be running.

Example:

```bash
source /opt/ros/humble/setup.bash
source ~/ws_3dof/install/setup.bash
ros2 launch jaco_moveit_config demo.launch.py
```

You do **not** need to actively use RViz to run the planners, but you do need the MoveIt services that `demo.launch.py` starts.

Quick check:

```bash
ros2 service list | grep check_state_validity
```

You should see:

```text
/check_state_validity
```

---

## Joint limits used

Current 6-DOF JACO joint limits:

- joint 1: `[-pi, pi]`
- joint 2: `[0, 2pi]`
- joint 3: `[0, 2pi]`
- joint 4: `[-pi, pi]`
- joint 5: `[-pi, pi]`
- joint 6: `[-pi, pi]`

The common default start used in the experiments is:

```text
q_start = [0.0, 3.142, 3.142, 0.0, 0.0, 0.0]
```

---

## Add planning-scene obstacles

This scene adds:

- a box acting like a shelf
- a cylinder obstacle
- a sphere obstacle

Run:

```bash
source /opt/ros/humble/setup.bash
source ~/ws_3dof/install/setup.bash

ros2 run qrrt_planner add_box_obstacle --remove --id box1
ros2 run qrrt_planner add_box_obstacle --remove --id sphere1
ros2 run qrrt_planner add_box_obstacle --remove --id cylinder1

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
```

Note: if you restart `move_group`, you need to re-add the obstacles.

---

## Run the classical grid-RRT

Baseline command:

```bash
source /opt/ros/humble/setup.bash
source ~/ws_3dof/install/setup.bash

ros2 run qrrt_planner run_crrt_grid \
  --bins-per-joint 10 \
  --q-start="0.0,3.142,3.142,0.0,0.0,0.0" \
  --q-goal="0.855,3.386,4.590,0.576,-1.623,1.518" \
  --max-iters 2000 \
  --candidate-count 32 \
  --top-k 8
```

With CSV logging:

```bash
ros2 run qrrt_planner run_crrt_grid \
  --bins-per-joint 10 \
  --q-start="0.0,3.142,3.142,0.0,0.0,0.0" \
  --q-goal="0.855,3.386,4.590,0.576,-1.623,1.518" \
  --max-iters 2000 \
  --candidate-count 32 \
  --top-k 8 \
  --csv-out ~/ws_3dof/results/crrt_grid_summary.csv \
  --debug-log ~/ws_3dof/results/crrt_grid_debug.csv
```

---

## Run the quantum grid-RRT

Current baseline quantum command:

```bash
source /opt/ros/humble/setup.bash
source ~/ws_3dof/install/setup.bash

ros2 run qrrt_planner run_qrrt_grid \
  --bins-per-joint 10 \
  --q-start="0.0,3.142,3.142,0.0,0.0,0.0" \
  --q-goal="0.855,3.386,4.590,0.576,-1.623,1.518" \
  --max-iters 2000 \
  --quantum-candidates 32 \
  --quantum-top-k 8 \
  --quantum-iters 1 \
  --quantum-shots 128
```

With CSV logging:

```bash
ros2 run qrrt_planner run_qrrt_grid \
  --bins-per-joint 10 \
  --q-start="0.0,3.142,3.142,0.0,0.0,0.0" \
  --q-goal="0.855,3.386,4.590,0.576,-1.623,1.518" \
  --max-iters 2000 \
  --quantum-candidates 32 \
  --quantum-top-k 8 \
  --quantum-iters 1 \
  --quantum-shots 128 \
  --csv-out ~/ws_3dof/results/qrrt_grid_summary.csv \
  --debug-log ~/ws_3dof/results/qrrt_grid_debug.csv
```

---

## Run grid A\*

Example:

```bash
source /opt/ros/humble/setup.bash
source ~/ws_3dof/install/setup.bash

ros2 run qrrt_planner run_grid_astar \
  --bins-per-joint 10 \
  --q-start="0.0,3.142,3.142,0.0,0.0,0.0" \
  --q-goal="0.855,3.386,4.590,0.576,-1.623,1.518"
```

Example benchmark form:

```bash
source /opt/ros/humble/setup.bash
source ~/ws_3dof/install/setup.bash

ros2 run qrrt_planner benchmark_grid_astar \
  --bins-per-joint 10 \
  --q-start="0.0,3.142,3.142,0.0,0.0,0.0" \
  --q-goal="0.855,3.386,4.590,0.576,-1.623,1.518" \
  --trials 1 \
  --scenario-name "grid_astar_test" \
  --csv-out ~/ws_3dof/results/grid_astar_test.csv
```

---

## Example obstacle scenarios

### Scenario 1 goal
From RViz joint values:

```text
[-103°, 180°, 104°, -84°, 88°, 127°]
```

Approximate radians:

```text
q_goal = [-1.798, 3.142, 1.815, -1.466, 1.536, 2.217]
```

### Scenario 2 goal

```text
[89°, 246°, 285°, 71°, 126°, 81°]
```

Approximate radians:

```text
q_goal = [1.553, 4.294, 4.974, 1.239, 2.199, 1.414]
```

### Scenario 3 goal

```text
[-82°, 240°, 287°, 129°, -141°, -88°]
```

Approximate radians:

```text
q_goal = [-1.431, 4.189, 5.009, 2.251, -2.461, -1.536]
```

---

## Example scenario commands

### Scenario 1
Classical:

```bash
ros2 run qrrt_planner run_crrt_grid \
  --bins-per-joint 10 \
  --q-start="0.0,3.142,3.142,0.0,0.0,0.0" \
  --q-goal="-1.798,3.142,1.815,-1.466,1.536,2.217" \
  --max-iters 2000 \
  --candidate-count 32 \
  --top-k 8
```

Quantum:

```bash
ros2 run qrrt_planner run_qrrt_grid \
  --bins-per-joint 10 \
  --q-start="0.0,3.142,3.142,0.0,0.0,0.0" \
  --q-goal="-1.798,3.142,1.815,-1.466,1.536,2.217" \
  --max-iters 2000 \
  --quantum-candidates 32 \
  --quantum-top-k 8 \
  --quantum-iters 1 \
  --quantum-shots 128
```

### Scenario 2
Classical:

```bash
ros2 run qrrt_planner run_crrt_grid \
  --bins-per-joint 10 \
  --q-start="0.0,3.142,3.142,0.0,0.0,0.0" \
  --q-goal="1.553,4.294,4.974,1.239,2.199,1.414" \
  --max-iters 2000 \
  --candidate-count 32 \
  --top-k 8
```

Quantum:

```bash
ros2 run qrrt_planner run_qrrt_grid \
  --bins-per-joint 10 \
  --q-start="0.0,3.142,3.142,0.0,0.0,0.0" \
  --q-goal="1.553,4.294,4.974,1.239,2.199,1.414" \
  --max-iters 2000 \
  --quantum-candidates 32 \
  --quantum-top-k 8 \
  --quantum-iters 1 \
  --quantum-shots 128
```

### Scenario 3
Classical:

```bash
ros2 run qrrt_planner run_crrt_grid \
  --bins-per-joint 10 \
  --q-start="0.0,3.142,3.142,0.0,0.0,0.0" \
  --q-goal="-1.431,4.189,5.009,2.251,-2.461,-1.536" \
  --max-iters 2000 \
  --candidate-count 32 \
  --top-k 8
```

Quantum:

```bash
ros2 run qrrt_planner run_qrrt_grid \
  --bins-per-joint 10 \
  --q-start="0.0,3.142,3.142,0.0,0.0,0.0" \
  --q-goal="-1.431,4.189,5.009,2.251,-2.461,-1.536" \
  --max-iters 2000 \
  --quantum-candidates 32 \
  --quantum-top-k 8 \
  --quantum-iters 1 \
  --quantum-shots 128
```

---

## Output metrics

The current runners report metrics such as:

- `success`
- `time (s)`
- `iterations`
- `nodes`
- `calls`
- `successful extensions`
- `invalid random targets`
- `empty candidate sets`
- `goal sample hits`
- `duplicate rejections`
- `invalid chosen state`
- `invalid chosen edge`
- `avg candidate count`
- `avg selected rank`
- `path waypoints`

These are meant to support apples-to-apples comparison between classical and quantum grid-RRT runs.

---

## Known issues

- The quantum runner currently prints many `Built Grover circuit with ... qubits` messages. This is debug spam from the oracle path and should eventually be gated behind a verbose flag.
- A valid continuous goal may snap to a different discrete grid cell.
- The planners depend on MoveIt being active; RViz alone is not sufficient unless `move_group` is running.
- Restarting MoveIt usually clears the planning scene, so obstacles may need to be re-added.
- Current quantum configurations are functional. Classical grid-RRT is generally faster and more stable overall, but quantum-guided grid-RRT showed promising performance in Scenario 3 at intermediate candidate settings.

---

## Recommended workflow

1. Launch MoveIt
2. Add obstacles
3. Pick a goal in RViz
4. Convert the joint values from degrees to radians
5. Run classical and quantum planners with the same start, goal, and grid resolution
6. Save outputs to CSV
7. Compare metrics across scenarios and seeds

---

## Notes

This repository is currently an experimental research codebase rather than a polished package. The focus is on testing discretized configuration-space planning and Grover-guided local sampling strategies for robotic motion planning.
