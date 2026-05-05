"""
run_qrrt_grid.py

Summary:
    Command-line runner for Quantum-guided RRT on the discretized joint-space grid.

    This script is the ROS 2 executable behind:

        ros2 run qrrt_planner run_qrrt_grid

    It:
        1. Parses command-line arguments.
        2. Builds the discretized joint-space grid.
        3. Connects to MoveIt for collision checking.
        4. Runs the quantum-guided grid-RRT planner.
        5. Prints planner metrics.
        6. Optionally appends summary results to a CSV file.
        7. Optionally writes a per-iteration debug trace.

    The actual qRRT planning logic lives in qrrt_grid.py. This file mainly
    handles experiment setup, timing, reporting, and CSV output.

    Important:
        The reported time is prototype/simulator wall-clock time. It should not
        be interpreted as real quantum hardware runtime.
"""

from __future__ import annotations

import argparse
import csv
import time
from pathlib import Path

import numpy as np
import rclpy

from qrrt_planner.grid_joint_space import make_joint_bins, total_grid_states
from qrrt_planner.moveit_collision_checker import MoveItCollisionChecker
from qrrt_planner.qrrt_grid import qrrt_grid


def parse_vector(text: str, dof: int, name: str) -> np.ndarray:
    """
    Parse a comma-separated joint vector from the command line.

    Args:
        text:
            Comma-separated joint values.
        dof:
            Expected number of joints.
        name:
            Argument name used in error messages.

    Returns:
        Joint vector as a NumPy array.
    """
    vals = [float(x.strip()) for x in text.split(",")]
    if len(vals) != dof:
        raise ValueError(f"{name} must have {dof} values")
    return np.array(vals, dtype=float)


def parse_joint_limits(text: str, dof: int):
    """
    Parse joint limits from the command line.

    Supports either:
        - one shared lo:hi pair for all joints
        - one lo:hi pair per joint

    Returns:
        List of (lower, upper) tuples.
    """
    parts = [p.strip() for p in text.split(",")]
    if len(parts) == 1:
        lo, hi = [float(x) for x in parts[0].split(":")]
        return [(lo, hi)] * dof

    if len(parts) != dof:
        raise ValueError("joint-limits must have either 1 pair or one pair per joint")

    out = []
    for p in parts:
        lo, hi = [float(x) for x in p.split(":")]
        out.append((lo, hi))
    return out


def parse_bins(text: str, dof: int):
    """
    Parse the number of discretization bins per joint.

    Supports either:
        - one value applied to all joints
        - one value per joint

    Returns:
        List of bin counts, one per joint.
    """
    parts = [int(x.strip()) for x in text.split(",")]
    if len(parts) == 1:
        return parts * dof

    if len(parts) != dof:
        raise ValueError("bins-per-joint must have either 1 value or one per joint")

    return parts


def append_csv_row(csv_path: str, row: dict) -> None:
    """
    Append one summary row to a CSV file.

    If the file does not exist yet, write the header first.
    """
    out = Path(csv_path)
    out.parent.mkdir(parents=True, exist_ok=True)
    exists = out.exists()

    with out.open("a", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(row.keys()))
        if not exists:
            writer.writeheader()
        writer.writerow(row)


def write_debug_trace(csv_path: str, trace: list[dict]) -> None:
    """
    Write the per-iteration debug trace to a CSV file.

    Unlike append_csv_row(), this overwrites the debug file each run.
    """
    if not trace:
        return

    out = Path(csv_path)
    out.parent.mkdir(parents=True, exist_ok=True)

    with out.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(trace[0].keys()))
        writer.writeheader()
        writer.writerows(trace)


def main():
    """
    Parse arguments, run quantum-guided grid-RRT, and report/save metrics.
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
    parser.add_argument("--max-iters", type=int, default=5000)
    parser.add_argument("--goal-sample-rate", type=float, default=0.10)
    parser.add_argument("--goal-radius-idx", type=float, default=0.0)
    parser.add_argument("--edge-step", type=float, default=0.05)
    parser.add_argument("--quantum-candidates", type=int, default=64)
    parser.add_argument("--quantum-top-k", type=int, default=8)
    parser.add_argument("--quantum-iters", type=int, default=1)
    parser.add_argument("--quantum-shots", type=int, default=64)
    parser.add_argument("--quantum-use-ibm", action="store_true")
    parser.add_argument("--quantum-backend", type=str, default="ibm_torino")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--csv-out", type=str, default=None)
    parser.add_argument("--debug-log", type=str, default=None)
    args = parser.parse_args()

    # Convert command-line strings into planner inputs.
    joint_limits = parse_joint_limits(args.joint_limits, args.dof)
    bins_per_joint = parse_bins(args.bins_per_joint, args.dof)
    q_start = parse_vector(args.q_start, args.dof, "q-start")
    q_goal = parse_vector(args.q_goal, args.dof, "q-goal")
    bins = make_joint_bins(joint_limits, bins_per_joint)

    rclpy.init()

    # MoveIt performs validity/collision checking for continuous joint states.
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

    # Time only the planner run, including MoveIt validity calls and quantum
    # simulation/hardware-call time made during planning.
    t0 = time.perf_counter()
    path_idx, stats = qrrt_grid(
        q_start=q_start,
        q_goal=q_goal,
        bins=bins,
        state_valid_fn=checker.is_state_valid,
        max_iters=args.max_iters,
        goal_sample_rate=args.goal_sample_rate,
        goal_radius_idx=args.goal_radius_idx,
        edge_step=args.edge_step,
        quantum_candidates=args.quantum_candidates,
        quantum_top_k=args.quantum_top_k,
        quantum_iters=args.quantum_iters,
        quantum_shots=args.quantum_shots,
        quantum_use_ibm=args.quantum_use_ibm,
        quantum_backend=args.quantum_backend,
        quantum_progress_weight=1.0,
        rng_seed=args.seed,
        debug=bool(args.debug_log),
    )
    dt = time.perf_counter() - t0

    # Average number of usable candidates per nonempty expansion.
    avg_candidate_count = 0.0
    if int(stats["candidate_count_nonempty_iters"]) > 0:
        avg_candidate_count = (
            float(stats["candidate_count_total"])
            / float(stats["candidate_count_nonempty_iters"])
        )

    # Average number of candidates marked as good for Grover.
    avg_good_set_size = 0.0
    if int(stats["quantum_calls"]) > 0:
        avg_good_set_size = (
            float(stats["good_set_size_total"])
            / float(stats["quantum_calls"])
        )

    # Average rank of the selected candidate by distance-to-goal.
    avg_selected_rank = 0.0
    if int(stats["quantum_calls"]) > 0:
        avg_selected_rank = (
            float(stats["selected_rank_total"])
            / float(stats["quantum_calls"])
        )

    # Console summary for quick inspection.
    print("Quantum RRT on discretized grid")
    print(f"dof:                     {args.dof}")
    print(f"joint limits:            {joint_limits}")
    print(f"bins per joint:          {bins_per_joint}")
    print(f"total grid states:       {total_grid_states(bins_per_joint)}")
    print(f"q_start:                 {q_start}")
    print(f"q_goal:                  {q_goal}")
    print(f"start_idx_raw:           {stats['start_idx_raw']}")
    print(f"goal_idx_raw:            {stats['goal_idx_raw']}")
    print(f"start_idx:               {stats['start_idx']}")
    print(f"goal_idx:                {stats['goal_idx']}")
    print(f"success:                 {stats['success']}")
    print(f"time (s):                {dt:.6f}")
    print(f"iterations:              {stats['iterations']}")
    print(f"nodes:                   {stats['nodes']}")
    print(f"quantum calls:           {stats['quantum_calls']}")
    print(f"quantum good extensions: {stats['quantum_successful_extensions']}")
    print(f"invalid random targets:  {stats['invalid_random_targets']}")
    print(f"empty candidate sets:    {stats['empty_candidate_sets']}")
    print(f"goal sample hits:        {stats['goal_sample_hits']}")
    print(f"duplicate rejections:    {stats['duplicate_rejections']}")
    print(f"invalid chosen state:    {stats['invalid_chosen_state_rejections']}")
    print(f"invalid chosen edge:     {stats['invalid_chosen_edge_rejections']}")
    print(f"avg candidate count:     {avg_candidate_count:.3f}")
    print(f"avg good-set size:       {avg_good_set_size:.3f}")
    print(f"avg selected rank:       {avg_selected_rank:.3f}")

    if path_idx is not None:
        print(f"path waypoints:          {len(path_idx)}")
        print(f"first idx:               {path_idx[0]}")
        print(f"last idx:                {path_idx[-1]}")
    else:
        print("path waypoints:          n/a")

    # Optional CSV summary row.
    if args.csv_out:
        row = {
            "planner_type": "qrrt_grid",
            "success": bool(stats["success"]),
            "time_sec": dt,
            "iterations": int(stats["iterations"]),
            "nodes": int(stats["nodes"]),
            "quantum_calls": int(stats["quantum_calls"]),
            "quantum_successful_extensions": int(stats["quantum_successful_extensions"]),
            "invalid_random_targets": int(stats["invalid_random_targets"]),
            "empty_candidate_sets": int(stats["empty_candidate_sets"]),
            "goal_sample_hits": int(stats["goal_sample_hits"]),
            "duplicate_rejections": int(stats["duplicate_rejections"]),
            "invalid_chosen_state_rejections": int(stats["invalid_chosen_state_rejections"]),
            "invalid_chosen_edge_rejections": int(stats["invalid_chosen_edge_rejections"]),
            "candidate_count_total": int(stats["candidate_count_total"]),
            "candidate_count_nonempty_iters": int(stats["candidate_count_nonempty_iters"]),
            "avg_candidate_count": avg_candidate_count,
            "good_set_size_total": int(stats["good_set_size_total"]),
            "avg_good_set_size": avg_good_set_size,
            "selected_rank_total": int(stats["selected_rank_total"]),
            "avg_selected_rank": avg_selected_rank,
            "path_waypoints": stats["path_waypoints"],
            "bins_per_joint": str(list(bins_per_joint)),
            "total_grid_states": total_grid_states(bins_per_joint),
            "q_start": ",".join(f"{x:.6f}" for x in q_start.tolist()),
            "q_goal": ",".join(f"{x:.6f}" for x in q_goal.tolist()),
            "start_idx_raw": str(stats["start_idx_raw"]),
            "goal_idx_raw": str(stats["goal_idx_raw"]),
            "start_idx": str(stats["start_idx"]),
            "goal_idx": str(stats["goal_idx"]),
            "seed": args.seed,
            "max_iters": args.max_iters,
            "goal_sample_rate": args.goal_sample_rate,
            "goal_radius_idx": args.goal_radius_idx,
            "quantum_candidates": args.quantum_candidates,
            "quantum_top_k": args.quantum_top_k,
            "quantum_iters": args.quantum_iters,
            "quantum_shots": args.quantum_shots,
            "quantum_use_ibm": args.quantum_use_ibm,
            "quantum_backend": args.quantum_backend,
        }
        append_csv_row(args.csv_out, row)

    # Optional per-iteration debug trace.
    if args.debug_log and stats["trace"] is not None:
        write_debug_trace(args.debug_log, stats["trace"])

    checker.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
