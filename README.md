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

---

## Candidate Sweeps

Candidate counts tested:

    32, 64, 128, 256, 512, 1024, 2048

Classical uses:

    --candidate-count

Quantum uses:

    --quantum-candidates

The requested candidate count is not always the usable candidate count. Invalid states, invalid edges, duplicates, and local grid limits reduce the actual candidate set.

Best observed quantum candidate settings so far:

    Scenario 1: k = 512
    Scenario 2: k = 256
    Scenario 3: k = 512

Common quantum settings used in the sweeps:

    quantum_top_k = 8
    quantum_iters = 1
    quantum_shots = 128

---

## Results

Completed experiment CSVs are stored in:

    results/

The results folder includes its own README:

    results/README.md

High-level current takeaways:

- A* works, but becomes slow in harder 6D grid scenarios.
- Classical grid-RRT is generally fast and stable.
- Quantum grid-RRT is less stable, but showed promising behavior in Scenario 3.
- The strongest observed result so far was Scenario 3 at k = 512:

    Classical grid-RRT: about 45.8 s
    Quantum grid-RRT:   about 17.6 s

This does not prove quantum speedup. It suggests that Grover-biased candidate selection may improve sampling behavior in some difficult planning cases.

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
    avg_selected_rank
    path_waypoints

Quantum-specific metrics include:

    avg_good_set_size
    quantum_candidates
    quantum_top_k
    quantum_iters
    quantum_shots

---

## Current Follow-Up Experiment

After the candidate-count sweeps, the next step is to run seed sweeps using the best observed candidate settings:

    Scenario 1: k = 512
    Scenario 2: k = 256
    Scenario 3: k = 512

Seed sweeps test whether the observed performance trends are stable across random trials.

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
