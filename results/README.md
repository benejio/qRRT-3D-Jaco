# Results Folder

This folder contains benchmark outputs for the 6-DOF JACO joint-space planning experiments.

Planners compared:

- `grid_astar`: systematic A* search over the discretized joint-space grid
- `crrt_grid`: classical grid-RRT
- `qrrt_grid`: quantum-guided grid-RRT using Grover-style candidate selection

## Grid

The JACO arm has 6 joints. With `--bins-per-joint 10`, the grid has:

    10^6 = 1,000,000 possible discrete joint states

Each grid state represents one full robot joint configuration. MoveIt checks whether sampled states and edges are collision-free.

## File Types

A* results:

    grid_astar_scenario_*.csv

Classical RRT sweeps:

    crrt_scenario_*_candidate_sweep.csv

Quantum RRT sweeps:

    qrrt_scenario_*_candidate_sweep.csv

Debug logs:

    *_debug.csv

Debug files contain per-iteration traces such as candidate count, selected rank, rejection reason, and tree node count.

## Scenarios

Common start state:

    q_start = [0.0, 3.142, 3.142, 0.0, 0.0, 0.0]

Scenario 1:

    q_goal = [-1.798, 3.142, 1.815, -1.466, 1.536, 2.217]

Scenario 2 adjusted goal:

    q_goal = [1.553, 4.294, 4.974, 1.239, 2.199, 1.414]

Scenario 3:

    q_goal = [-1.431, 4.189, 5.009, 2.251, -2.461, -1.536]

## Candidate Sweeps

Candidate counts tested:

    32, 64, 128, 256, 512, 1024, 2048

Requested candidate count is not always the usable candidate count. Invalid states, invalid edges, duplicates, and local grid limits reduce the actual candidate set.

## Current Takeaways

- A* works, but becomes slow in harder 6D grid scenarios.
- Classical grid-RRT is generally fast and stable.
- Quantum grid-RRT is less stable, but showed promising behavior in Scenario 3.
- The strongest result so far was Scenario 3 at `k=512`:

    Classical grid-RRT: about 45.8 s  
    Quantum grid-RRT:   about 17.6 s

This does not prove quantum speedup. The current quantum runs use Qiskit simulation unless `--quantum-use-ibm` is enabled. Simulator wall-clock time should be treated as prototype runtime, not hardware quantum runtime.

## Next Step

Run seed sweeps on the best candidate settings:

    Scenario 1: k = 512
    Scenario 2: k = 256
    Scenario 3: k = 512

This will show whether the observed trends are stable across random seeds.
