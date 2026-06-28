# QRRT Planner on Discretized Joint-Space Grids

This repository contains a reproducible ROS 2 / MoveIt workspace for testing motion planning on a 6-DOF Kinova JACO arm.

The current focus is comparing:

- Classical RRT on a discretized joint-space grid
- Quantum-guided RRT on a discretized joint-space grid
- Grid-based A* on the same discretized configuration space

The planners search in a discretized 6D joint configuration space. Collision checking is performed through MoveIt using the continuous robot model and planning-scene obstacles.

---

## Repository Layout

This repository root is intended to be the ROS 2 workspace root.

ROS packages are located in:

    src/

Included packages:

    src/jaco_description
    src/jaco_moveit_config
    src/kinova_description
    src/qrrt_planner

This repository intentionally includes the JACO description and MoveIt configuration packages so the workspace can be cloned, built, and run without separately generating a MoveIt setup.

Generated ROS folders are excluded:

    build/
    install/
    log/
    
---

## Main Planner Files

Main Python files under src/qrrt_planner/qrrt_planner/:

- moveit_collision_checker.py
  Connects to MoveIt and checks robot state validity.

- grid_joint_space.py
  Builds the discretized joint-space grid and provides A* search utilities.

- crrt_grid.py
  Classical RRT over the discretized joint-space grid.

- qrrt_grid.py
  Quantum-guided RRT over the discretized joint-space grid.

- grid_quantum_sampler.py
  Selects candidates using Grover-style probability weighting.

- oracle_qiskit.py
  Builds and runs Grover circuits using Qiskit.

- add_box_obstacle.py
  Adds or removes box, sphere, and cylinder obstacles in the MoveIt planning scene.

- run_crrt_grid.py
  Command-line runner for classical grid-RRT.

- run_qrrt_grid.py
  Command-line runner for quantum grid-RRT.

- run_grid_astar.py
  Command-line runner for a single grid A* search.

- benchmark_grid_astar.py
  Repeated A* benchmark runner with CSV output.

---

## System Requirements

Tested environment:

    Ubuntu 22.04
    ROS 2 Humble
    MoveIt 2
    Python 3.10
    Qiskit

---

## Clone and Build

Clone the repository into a folder named ws_3dof:

    cd ~
    git clone https://github.com/benejio/qRRT-3D-Jaco.git ws_3dof
    cd ~/ws_3dof

Build the full workspace:

    source /opt/ros/humble/setup.bash
    colcon build --symlink-install
    source ~/ws_3dof/install/setup.bash

Check that the packages are available:

    ros2 pkg list | grep -E "qrrt_planner|jaco_description|jaco_moveit_config|kinova_description"

---

## Python Dependencies

Install Python dependencies with:

    pip install -r requirements.txt

This only installs the Python/Qiskit dependencies. ROS 2 Humble and MoveIt 2 must still be installed through the normal ROS installation process.

---

## Launch MoveIt

Before running any planner, move_group must be running.

In one terminal:

    source /opt/ros/humble/setup.bash
    source ~/ws_3dof/install/setup.bash
    ros2 launch jaco_moveit_config demo.launch.py

You do not need to actively use RViz to run the planners, but this launch file starts the MoveIt services needed by the planners.

In another terminal, verify the state-validity service:

    source /opt/ros/humble/setup.bash
    source ~/ws_3dof/install/setup.bash
    ros2 service list | grep check_state_validity

Expected output:

    /check_state_validity

---

## Joint-Space Grid

The robot has 6 joints:

    q = [q1, q2, q3, q4, q5, q6]

Current joint limits:

    joint 1: [-pi, pi]
    joint 2: [0, 2pi]
    joint 3: [0, 2pi]
    joint 4: [-pi, pi]
    joint 5: [-pi, pi]
    joint 6: [-pi, pi]

With:

    --bins-per-joint 10

the grid contains:

    10^6 = 1,000,000 possible discrete joint states

Each grid state represents one full robot joint configuration. MoveIt checks whether sampled states and edges are collision-free.

Common start state used in the experiments:

    q_start = [0.0, 3.142, 3.142, 0.0, 0.0, 0.0]

---

## Add Planning-Scene Obstacles

If MoveIt is restarted, obstacles usually need to be re-added.

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

### Add Scene 4 Obstacles

Scene 4 uses the same shelf location, moves the cylinder to the center of the shelf, and places the sphere on the `+y` side of the cylinder.

    ros2 run qrrt_planner add_box_obstacle --remove --id box1
    ros2 run qrrt_planner add_box_obstacle --remove --id cylinder1
    ros2 run qrrt_planner add_box_obstacle --remove --id sphere1

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

---

## Run Classical Grid-RRT

    ros2 run qrrt_planner run_crrt_grid \
      --bins-per-joint 10 \
      --q-start="0.0,3.142,3.142,0.0,0.0,0.0" \
      --q-goal="-1.431,4.189,5.009,2.251,-2.461,-1.536" \
      --max-iters 2000 \
      --candidate-count 512 \
      --top-k 8 \
      --seed 1

With CSV logging:

    ros2 run qrrt_planner run_crrt_grid \
      --bins-per-joint 10 \
      --q-start="0.0,3.142,3.142,0.0,0.0,0.0" \
      --q-goal="-1.431,4.189,5.009,2.251,-2.461,-1.536" \
      --max-iters 2000 \
      --candidate-count 512 \
      --top-k 8 \
      --seed 1 \
      --csv-out ~/ws_3dof/results/crrt_test.csv \
      --debug-log ~/ws_3dof/results/crrt_test_debug.csv

---

## Run Quantum Grid-RRT

    ros2 run qrrt_planner run_qrrt_grid \
      --bins-per-joint 10 \
      --q-start="0.0,3.142,3.142,0.0,0.0,0.0" \
      --q-goal="-1.431,4.189,5.009,2.251,-2.461,-1.536" \
      --max-iters 2000 \
      --quantum-candidates 512 \
      --quantum-top-k 8 \
      --quantum-iters 1 \
      --quantum-shots 128 \
      --seed 1

With CSV logging:

    ros2 run qrrt_planner run_qrrt_grid \
      --bins-per-joint 10 \
      --q-start="0.0,3.142,3.142,0.0,0.0,0.0" \
      --q-goal="-1.431,4.189,5.009,2.251,-2.461,-1.536" \
      --max-iters 2000 \
      --quantum-candidates 512 \
      --quantum-top-k 8 \
      --quantum-iters 1 \
      --quantum-shots 128 \
      --seed 1 \
      --csv-out ~/ws_3dof/results/qrrt_test.csv \
      --debug-log ~/ws_3dof/results/qrrt_test_debug.csv

Important: the current quantum path uses Qiskit simulation unless --quantum-use-ibm is enabled. Simulator wall-clock time is prototype runtime, not real quantum hardware runtime.

---

## Run Grid A*

Single A* run:

    ros2 run qrrt_planner run_grid_astar \
      --bins-per-joint 10 \
      --q-start="0.0,3.142,3.142,0.0,0.0,0.0" \
      --q-goal="-1.431,4.189,5.009,2.251,-2.461,-1.536"

Benchmark form with CSV output:

    ros2 run qrrt_planner benchmark_grid_astar \
      --bins-per-joint 10 \
      --q-start="0.0,3.142,3.142,0.0,0.0,0.0" \
      --q-goal="-1.431,4.189,5.009,2.251,-2.461,-1.536" \
      --trials 1 \
      --scenario-name "scenario_3" \
      --csv-out ~/ws_3dof/results/grid_astar_scenario_3.csv

---

## Experiment Scenarios

### Scenario 1

RViz degrees:

    [-103 deg, 180 deg, 104 deg, -84 deg, 88 deg, 127 deg]

Radians:

    q_goal = [-1.798, 3.142, 1.815, -1.466, 1.536, 2.217]

### Scenario 2

The original Scenario 2 goal was invalid, so this adjusted valid goal was used.

RViz degrees:

    [89 deg, 246 deg, 285 deg, 71 deg, 126 deg, 81 deg]

Radians:

    q_goal = [1.553, 4.294, 4.974, 1.239, 2.199, 1.414]

### Scenario 3

RViz degrees:

    [-82 deg, 240 deg, 287 deg, 129 deg, -141 deg, -88 deg]

Radians:

    q_goal = [-1.431, 4.189, 5.009, 2.251, -2.461, -1.536]

### Scenario 4

Scene 4 keeps the shelf in the same position as the earlier scenes, places the cylinder at the center of the shelf, and places the spherical obstacle on the `+y` side of the cylinder. The start and goal configurations are selected on the opposite, non-ball side of the cylinder.

Obstacle layout:

    shelf/box: center = (0.25, 0.00, 0.38), size = (0.30, 0.95, 0.10)
    cylinder:  center = (0.25, 0.00, 0.58), radius = 0.07, height = 0.30
    sphere:    center = (0.25, 0.25, 0.52), radius = 0.09

Start RViz degrees:

    [-120 deg, 183 deg, 97 deg, -73 deg, 64 deg, 135 deg]

Start radians:

    q_start = [-2.094, 3.194, 1.693, -1.274, 1.117, 2.356]

Goal RViz degrees:

    [-95 deg, 180 deg, 267 deg, 117 deg, -136 deg, -135 deg]

Goal radians:

    q_goal = [-1.658, 3.142, 4.660, 2.042, -2.374, -2.356]

---

## Candidate Sweeps

Candidate counts tested:

    32, 64, 128, 256, 512, 1024, 2048

Classical uses:

    --candidate-count

Quantum uses:

    --quantum-candidates

The requested candidate count is not always the usable candidate count. Invalid states, invalid edges, candidates already present in the tree, duplicates, and local grid limits reduce the actual candidate set before it is sent to the quantum sampler. The resulting database is randomized before basis-state encoding so Grover receives an unordered valid-candidate list.

Best single successful Grover-guided candidate settings observed so far:

    Scenario 1: k_c = 512
    Scenario 2: k_c = 256
    Scenario 3: k_c = 128

Common quantum settings used in the sweeps:

    quantum_iters = 1
    quantum_shots = 128
    quantum_best_rounds = 3
    quantum_target_weight = 1.0
    quantum_goal_weight = 1.0

The older `quantum_top_k` setting is deprecated for qRRT. The qRRT implementation no longer classically ranks the candidate list and marks the top-k candidates.

---

## Current qRRT Candidate-Selection Behavior

The qRRT planner now separates candidate usability from quantum selection:

1. ROS 2 / MoveIt performs collision and edge checking.
2. The planner builds a valid local candidate database and removes candidates already in the tree.
3. The valid candidate list is shuffled so the database is unordered.
4. The quantum sampler starts from a random incumbent candidate.
5. Grover marks candidates whose local extension score beats the incumbent and samples a better candidate.
6. This improvement search repeats for `quantum_best_rounds`.

The local extension score rewards progress toward the current RRT random target and the final goal. This makes Grover responsible for searching the unordered valid list for a better expansion candidate, rather than merely sampling from a classically sorted top-k list.

---

## Results

Completed experiment CSVs are stored in:

    results/

The results folder includes its own README:

    results/README.md

High-level current takeaways:

- A* works, but becomes slow in harder 6D grid scenarios.
- Classical grid-RRT is generally more reliable across the 20-seed Scenario 3 sweep.
- Grover-guided grid-RRT is less stable overall, but it can require fewer iterations among successful runs in some candidate-count settings.
- Scenario 3 showed candidate-set saturation: increasing the requested candidate count did not always increase the usable candidate pool after filtering.

These results do not prove quantum speedup. They suggest that Grover-guided candidate selection can influence sampling behavior in some difficult planning cases, but its benefit depends on candidate-set construction, marked-subset quality, and random seed.

---

## Output Metrics

The runner scripts report metrics such as:

    success
    time_sec
    iterations
    nodes
    classical_calls / quantum_calls
    successful_extensions
    invalid_random_targets
    empty_candidate_sets
    goal_sample_hits
    duplicate_rejections
    invalid_chosen_state_rejections
    invalid_chosen_edge_rejections
    avg_candidate_count
    path_waypoints

Classical grid-RRT still reports `avg_selected_rank` for its top-k baseline.

Quantum-specific metrics include:

    avg_marked_set_size
    grover_rounds_total
    grover_improvements
    avg_initial_score
    avg_selected_score
    quantum_candidates
    quantum_iters
    quantum_shots
    quantum_best_rounds
    quantum_target_weight
    quantum_goal_weight

---

## Current Follow-Up Experiment

The current follow-up experiment is Scenario 4, which tests a shelf scene where the cylinder is centered on the shelf, the sphere is placed on the +y side of the cylinder, and the start/goal configurations are on the non-ball side.

Scenario 4 is evaluated over 20 random seeds and candidate counts:

    32, 64, 128, 256, 512, 1024, 2048

The outputs are written to:

    results/crrt_scenario_4_candidate_seed_sweep.csv
    results/qrrt_scenario_4_candidate_seed_sweep.csv

---

## Known Issues

- The quantum runner currently prints many "Built Grover circuit with ... qubits" messages.
- A valid continuous goal may snap to a different discrete grid cell.
- The planners depend on MoveIt being active.
- Restarting MoveIt usually clears the planning scene, so obstacles may need to be re-added.
- Simulator wall-clock time should not be interpreted as quantum hardware runtime.
- Current quantum results are promising but not yet proof of quantum speedup.

---

## Recommended Workflow

1. Clone the repository as ~/ws_3dof.
2. Build and source the workspace.
3. Launch MoveIt.
4. Add the obstacle scene.
5. Pick or verify a goal pose in RViz.
6. Run A*, classical grid-RRT, and quantum grid-RRT.
7. Save outputs to CSV.
8. Compare success, runtime, nodes, calls, candidate counts, and path waypoints.

---

## Notes

This repository is an experimental research codebase. The purpose is to evaluate discretized configuration-space planning and Grover-guided local sampling strategies for robotic motion planning on a 6-DOF manipulator.
