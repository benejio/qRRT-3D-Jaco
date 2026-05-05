"""
benchmark_grid_astar.py

Summary:
    Runs repeated grid-based A* planning trials in the discretized 6-DOF
    joint space of the JACO arm.

    The script converts continuous start and goal joint states into grid
    indices, calls the A* planner from grid_joint_space.py, records planner
    metrics, and optionally appends the results to a CSV file.

    This file is mainly used as a baseline comparison against:
        - Classical grid-RRT
        - Quantum-guided grid-RRT

    Unlike RRT, A* searches the grid more systematically. This makes it useful
    as a reference baseline, but it can become expensive because the full
    discretized 6D grid can contain many states.

    Example:
        ros2 run qrrt_planner benchmark_grid_astar \
            --bins-per-joint 10 \
            --q-start="0.0,3.142,3.142,0.0,0.0,0.0" \
            --q-goal="-1.431,4.189,5.009,2.251,-2.461,-1.536" \
            --trials 1 \
            --scenario-name "scenario_3" \
            --csv-out ~/ws_3dof/results/grid_astar_scenario_3.csv
"""

from __future__ import annotations

import argparse
import csv
import random
import time
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np
import rclpy

from qrrt_planner.grid_joint_space import (
    astar_grid_joint_space,
    grid_path_to_q_path,
    make_joint_bins,
    q_to_grid,
    total_grid_states,
    edge_is_valid_q,
)
from qrrt_planner.moveit_collision_checker import MoveItCollisionChecker


ArrayLike = Sequence[float]


def parse_vector(text: str, dof: int, name: str) -> np.ndarray:
    """
    Convert a comma-separated joint vector string into a NumPy array.

    Args:
        text:
            Comma-separated joint values, such as "0.0,3.142,3.142,0.0,0.0,0.0".
        dof:
            Expected number of joints.
        name:
            Name used in error messages.

    Returns:
        Joint vector as a NumPy array.

    Raises:
        ValueError:
            If the vector does not contain exactly dof values.
    """
    vals = [float(x.strip()) for x in text.split(",")]
    if len(vals) != dof:
        raise ValueError(f"{name} must have exactly {dof} comma-separated values")
    return np.array(vals, dtype=float)


def parse_joint_limits(text: str, dof: int) -> List[Tuple[float, float]]:
    """
    Parse joint limits from the command line.

    Supports either:
        - one shared limit pair for all joints, such as "-3.14:3.14"
        - one limit pair per joint

    Returns:
        List of (lower, upper) joint-limit tuples.
    """
    parts = [p.strip() for p in text.split(",")]

    if len(parts) == 1:
        lo, hi = [float(x) for x in parts[0].split(":")]
        return [(lo, hi)] * dof

    if len(parts) != dof:
        raise ValueError(
            "joint-limits must contain either one lo:hi pair or one pair per joint"
        )

    joint_limits: List[Tuple[float, float]] = []
    for p in parts:
        lo, hi = [float(x) for x in p.split(":")]
        joint_limits.append((lo, hi))
    return joint_limits


def parse_bins(text: str, dof: int) -> List[int]:
    """
    Parse the number of grid bins per joint.

    Supports either:
        - one value applied to all joints, such as "10"
        - one value per joint, such as "10,10,10,10,10,10"

    Returns:
        List containing one bin count per joint.
    """
    parts = [int(x.strip()) for x in text.split(",")]

    if len(parts) == 1:
        return parts * dof

    if len(parts) != dof:
        raise ValueError(
            "bins-per-joint must contain either one integer or one integer per joint"
        )

    return parts


def vector_to_str(q: np.ndarray) -> str:
    """
    Convert a NumPy joint vector into a compact CSV-safe string.
    """
    return ",".join(f"{x:.6f}" for x in q.tolist())


def sample_uniform_joint_vector(
    joint_limits: Sequence[Tuple[float, float]],
    rng: np.random.Generator,
) -> np.ndarray:
    """
    Randomly sample one continuous joint-space configuration.

    Each joint is sampled uniformly within its joint limits.
    """
    return np.array(
        [rng.uniform(lo, hi) for (lo, hi) in joint_limits],
        dtype=float,
    )


def sample_valid_scenario(
    joint_limits: Sequence[Tuple[float, float]],
    state_valid_fn,
    rng: np.random.Generator,
    difficulty: str = "medium",
    edge_step: float = 0.05,
    max_tries: int = 1000,
) -> Tuple[np.ndarray, np.ndarray]:
    """
    Generate a random valid start/goal planning scenario.

    This is only used when --randomize-scenarios is enabled. It samples start
    and goal configurations until both are valid and the distance between them
    matches the requested difficulty.

    For easy cases, the direct edge must be valid.
    For medium/hard cases, the direct edge must be invalid so the planner has
    to search instead of taking a straight-line joint-space path.
    """
    if difficulty == "easy":
        d_min, d_max = 1.0, 2.5
        require_direct_edge_valid = True
    elif difficulty == "medium":
        d_min, d_max = 2.0, 4.0
        require_direct_edge_valid = False
    elif difficulty == "hard":
        d_min, d_max = 3.5, 6.0
        require_direct_edge_valid = False
    else:
        raise ValueError(f"Unknown difficulty: {difficulty}")

    for _ in range(max_tries):
        q_start = sample_uniform_joint_vector(joint_limits, rng)
        q_goal = sample_uniform_joint_vector(joint_limits, rng)

        # Both endpoint states must be collision-free.
        if not state_valid_fn(q_start):
            continue
        if not state_valid_fn(q_goal):
            continue

        # Filter by start-to-goal distance to control scenario difficulty.
        d = float(np.linalg.norm(q_start - q_goal))
        if not (d_min <= d <= d_max):
            continue

        # Check whether the straight-line joint-space edge is valid.
        direct_edge_valid = edge_is_valid_q(
            q_start,
            q_goal,
            state_valid_fn,
            edge_step,
        )

        if require_direct_edge_valid and direct_edge_valid:
            return q_start, q_goal

        if (not require_direct_edge_valid) and (not direct_edge_valid):
            return q_start, q_goal

    raise RuntimeError(
        f"Could not generate a valid {difficulty} scenario in {max_tries} tries"
    )


def run_trials(
    scenario_name: str,
    q_start: np.ndarray,
    q_goal: np.ndarray,
    joint_limits: Sequence[Tuple[float, float]],
    bins_per_joint: Sequence[int],
    state_valid_fn,
    trials: int,
    edge_step: float,
    seed: int,
    csv_out: Optional[str],
    randomize_scenarios: bool = False,
    difficulty: str = "medium",
) -> None:
    """
    Run one or more A* planning trials and optionally write metrics to CSV.

    Args:
        scenario_name:
            Label stored in the output CSV.
        q_start, q_goal:
            Continuous start and goal joint vectors.
        joint_limits:
            Joint limits used to build the grid.
        bins_per_joint:
            Number of discretization bins for each joint.
        state_valid_fn:
            Function that returns True if a continuous joint state is valid.
        trials:
            Number of A* trials to run.
        edge_step:
            Step size used when checking edge validity.
        seed:
            Base random seed. Trial i uses seed + i.
        csv_out:
            Optional CSV output path.
        randomize_scenarios:
            If True, generate a new valid start/goal pair each trial.
        difficulty:
            Difficulty level used only when randomize_scenarios is True.
    """
    bins = make_joint_bins(joint_limits, bins_per_joint)

    success_count = 0
    times_sec: List[float] = []
    expanded_nodes: List[int] = []
    visited_nodes: List[int] = []
    invalid_states: List[int] = []
    invalid_edges: List[int] = []
    path_length_idx: List[int] = []
    path_length_q: List[float] = []
    snap_start_error: List[float] = []
    snap_goal_error: List[float] = []
    rows: List[Dict[str, object]] = []

    print("Grid A* joint-space benchmark")
    print(f"dof:              {len(joint_limits)}")
    print(f"joint limits:     {joint_limits}")
    print(f"bins per joint:   {list(bins_per_joint)}")
    print(f"total grid states:{total_grid_states(bins_per_joint)}")
    print(f"q_start:          {q_start}")
    print(f"q_goal:           {q_goal}")
    print(f"trials:           {trials}")
    print(f"edge_step:        {edge_step}")
    print(f"randomized:       {randomize_scenarios}")
    if randomize_scenarios:
        print(f"difficulty:       {difficulty}")
    if csv_out:
        print(f"csv_out:          {csv_out}")

    for trial in range(trials):
        trial_seed = seed + trial
        random.seed(trial_seed)
        np.random.seed(trial_seed)
        rng = np.random.default_rng(trial_seed)

        q_start_trial = q_start.copy()
        q_goal_trial = q_goal.copy()

        # Optionally replace the provided start/goal with a random valid pair.
        if randomize_scenarios:
            q_start_trial, q_goal_trial = sample_valid_scenario(
                joint_limits=joint_limits,
                state_valid_fn=state_valid_fn,
                rng=rng,
                difficulty=difficulty,
                edge_step=edge_step,
            )

        # Convert continuous joint vectors to grid indices and snapped grid states.
        start_idx = q_to_grid(q_start_trial, bins)
        goal_idx = q_to_grid(q_goal_trial, bins)
        q_start_snap = np.array(
            [bins[d][start_idx[d]] for d in range(len(start_idx))],
            dtype=float,
        )
        q_goal_snap = np.array(
            [bins[d][goal_idx[d]] for d in range(len(goal_idx))],
            dtype=float,
        )

        # Run A* and time the search.
        t0 = time.perf_counter()
        path_idx, stats = astar_grid_joint_space(
            q_start=q_start_trial,
            q_goal=q_goal_trial,
            joint_limits=joint_limits,
            bins_per_joint=bins_per_joint,
            state_valid_fn=state_valid_fn,
            edge_step=edge_step,
        )
        dt = time.perf_counter() - t0

        # Store metrics from this trial.
        times_sec.append(dt)
        expanded_nodes.append(int(stats["expanded_nodes"]))
        visited_nodes.append(int(stats["visited_nodes"]))
        invalid_states.append(int(stats["invalid_neighbor_states"]))
        invalid_edges.append(int(stats["invalid_neighbor_edges"]))
        snap_start_error.append(float(np.linalg.norm(q_start_trial - q_start_snap)))
        snap_goal_error.append(float(np.linalg.norm(q_goal_trial - q_goal_snap)))

        success = bool(stats["success"])
        if success:
            success_count += 1
            path_length_idx.append(int(stats["path_length_idx"]))
            path_length_q.append(float(stats["path_length_q"]))

        rows.append(
            {
                "scenario_name": scenario_name,
                "trial": trial,
                "seed": trial_seed,
                "success": success,
                "time_sec": dt,
                "expanded_nodes": int(stats["expanded_nodes"]),
                "visited_nodes": int(stats["visited_nodes"]),
                "invalid_neighbor_states": int(stats["invalid_neighbor_states"]),
                "invalid_neighbor_edges": int(stats["invalid_neighbor_edges"]),
                "path_length_idx": stats["path_length_idx"],
                "path_length_q": stats["path_length_q"],
                "q_start": vector_to_str(q_start_trial),
                "q_goal": vector_to_str(q_goal_trial),
                "start_idx": str(stats["start_idx"]),
                "goal_idx": str(stats["goal_idx"]),
                "start_q_snapped": vector_to_str(
                    np.asarray(stats["start_q_snapped"], dtype=float)
                ),
                "goal_q_snapped": vector_to_str(
                    np.asarray(stats["goal_q_snapped"], dtype=float)
                ),
                "snap_start_error": float(
                    np.linalg.norm(
                        q_start_trial - np.asarray(stats["start_q_snapped"], dtype=float)
                    )
                ),
                "snap_goal_error": float(
                    np.linalg.norm(
                        q_goal_trial - np.asarray(stats["goal_q_snapped"], dtype=float)
                    )
                ),
                "bins_per_joint": str(list(bins_per_joint)),
                "total_grid_states": total_grid_states(bins_per_joint),
            }
        )

    print()
    print(f"success:          {success_count}/{trials} ({100.0 * success_count / trials:.1f}%)")
    print(f"avg time (s):     {np.mean(times_sec):.6f}")
    print(f"avg expanded:     {np.mean(expanded_nodes):.2f}")
    print(f"avg visited:      {np.mean(visited_nodes):.2f}")
    print(f"avg invalid state:{np.mean(invalid_states):.2f}")
    print(f"avg invalid edge: {np.mean(invalid_edges):.2f}")
    print(f"avg snap err s:   {np.mean(snap_start_error):.6f}")
    print(f"avg snap err g:   {np.mean(snap_goal_error):.6f}")

    if path_length_idx:
        print(f"avg path idx len: {np.mean(path_length_idx):.2f}")
        print(f"avg path q len:   {np.mean(path_length_q):.6f}")
    else:
        print("avg path idx len: n/a")
        print("avg path q len:   n/a")

    # Append trial rows to CSV if requested.
    if csv_out:
        out_path = Path(csv_out)
        out_path.parent.mkdir(parents=True, exist_ok=True)

        file_exists = out_path.exists()
        fieldnames = list(rows[0].keys()) if rows else []

        with out_path.open("a", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            if not file_exists:
                writer.writeheader()
            writer.writerows(rows)


def main():
    """
    Command-line entry point for the A* benchmark runner.

    This function parses arguments, starts ROS, creates the MoveIt collision
    checker, validates the requested start/goal states, runs the trials, and
    shuts ROS down cleanly.
    """
    parser = argparse.ArgumentParser()
    parser.add_argument("--dof", type=int, default=6)
    parser.add_argument(
        "--joint-limits",
        type=str,
        default="-3.1416:3.1416,0.0:6.2832,0.0:6.2832,-3.1416:3.1416,-3.1416:3.1416,-3.1416:3.1416",
    )
    parser.add_argument("--bins-per-joint", type=str, default="10")
    parser.add_argument("--q-start", type=str, required=True)
    parser.add_argument("--q-goal", type=str, required=True)
    parser.add_argument("--trials", type=int, default=1)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--edge-step", type=float, default=0.05)
    parser.add_argument("--randomize-scenarios", action="store_true")
    parser.add_argument("--difficulty", type=str, default="medium")
    parser.add_argument("--scenario-name", type=str, default="grid_astar")
    parser.add_argument("--csv-out", type=str, default=None)
    args = parser.parse_args()

    joint_limits = parse_joint_limits(args.joint_limits, args.dof)
    bins_per_joint = parse_bins(args.bins_per_joint, args.dof)
    q_start = parse_vector(args.q_start, args.dof, "q-start")
    q_goal = parse_vector(args.q_goal, args.dof, "q-goal")

    rclpy.init()

    checker = MoveItCollisionChecker(
        group_name="arm",
        joint_names=[
            "j2n6s300_joint_1",
            "j2n6s300_joint_2",
            "j2n6s300_joint_3",
            "j2n6s300_joint_4",
            "j2n6s300_joint_5",
            "j2n6s300_joint_6",
        ],
    )

    start_valid = checker.is_state_valid(q_start)
    goal_valid = checker.is_state_valid(q_goal)

    print(f"start_valid:      {start_valid}")
    print(f"goal_valid:       {goal_valid}")

    # For fixed scenarios, fail early if the start or goal is invalid.
    # Randomized scenarios are checked inside sample_valid_scenario().
    if (not args.randomize_scenarios) and (not start_valid):
        checker.destroy_node()
        rclpy.shutdown()
        raise ValueError("Start state is invalid")

    if (not args.randomize_scenarios) and (not goal_valid):
        checker.destroy_node()
        rclpy.shutdown()
        raise ValueError("Goal state is invalid")

    run_trials(
        scenario_name=args.scenario_name,
        q_start=q_start,
        q_goal=q_goal,
        joint_limits=joint_limits,
        bins_per_joint=bins_per_joint,
        state_valid_fn=checker.is_state_valid,
        trials=args.trials,
        edge_step=args.edge_step,
        seed=args.seed,
        csv_out=args.csv_out,
        randomize_scenarios=args.randomize_scenarios,
        difficulty=args.difficulty,
    )

    checker.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
