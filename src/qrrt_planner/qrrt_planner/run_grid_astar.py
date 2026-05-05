"""
run_grid_astar.py

Summary:
    Command-line runner for a single Grid A* search in discretized joint space.

    This script is the ROS 2 executable behind:

        ros2 run qrrt_planner run_grid_astar

    It:
        1. Parses start/goal, joint limits, and grid resolution.
        2. Connects to MoveIt for collision checking.
        3. Runs A* over the discretized joint-space grid.
        4. Prints search metrics and path information.

    This is useful for quickly testing one A* planning query. For repeated
    trials and CSV output, use benchmark_grid_astar.py instead.
"""

import argparse
import time

import numpy as np
import rclpy

from qrrt_planner.grid_joint_space import (
    astar_grid_joint_space,
    grid_path_to_q_path,
    total_grid_states,
)
from qrrt_planner.moveit_collision_checker import MoveItCollisionChecker


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
    Parse the number of grid bins per joint.

    Supports either:
        - one bin count applied to every joint
        - one bin count per joint

    Returns:
        List of bin counts, one per joint.
    """
    parts = [int(x.strip()) for x in text.split(",")]
    if len(parts) == 1:
        return parts * dof

    if len(parts) != dof:
        raise ValueError("bins-per-joint must have either 1 value or one per joint")

    return parts


def main():
    """
    Parse arguments, run Grid A*, and print the resulting search metrics.
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
    parser.add_argument("--edge-step", type=float, default=0.05)
    args = parser.parse_args()

    # Convert command-line strings into planner inputs.
    joint_limits = parse_joint_limits(args.joint_limits, args.dof)
    bins_per_joint = parse_bins(args.bins_per_joint, args.dof)
    q_start = parse_vector(args.q_start, args.dof, "q-start")
    q_goal = parse_vector(args.q_goal, args.dof, "q-goal")

    rclpy.init()

    # MoveIt performs collision checking for the continuous robot model.
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

    # Run A* and time the search.
    t0 = time.perf_counter()
    path_idx, stats = astar_grid_joint_space(
        q_start=q_start,
        q_goal=q_goal,
        joint_limits=joint_limits,
        bins_per_joint=bins_per_joint,
        state_valid_fn=checker.is_state_valid,
        edge_step=args.edge_step,
    )
    dt = time.perf_counter() - t0

    # Print search summary.
    print("Grid A* joint-space search")
    print(f"dof:              {args.dof}")
    print(f"joint limits:     {joint_limits}")
    print(f"bins per joint:   {bins_per_joint}")
    print(f"total grid states:{total_grid_states(bins_per_joint)}")
    print(f"q_start:          {q_start}")
    print(f"q_goal:           {q_goal}")
    print(f"start_idx:        {stats['start_idx']}")
    print(f"goal_idx:         {stats['goal_idx']}")
    print(f"success:          {stats['success']}")
    print(f"time (s):         {dt:.6f}")
    print(f"expanded_nodes:   {stats['expanded_nodes']}")
    print(f"visited_nodes:    {stats['visited_nodes']}")
    print(f"invalid states:   {stats['invalid_neighbor_states']}")
    print(f"invalid edges:    {stats['invalid_neighbor_edges']}")
    print(f"path_length_idx:  {stats['path_length_idx']}")
    print(f"path_length_q:    {stats['path_length_q']}")

    # If a path was found, convert grid indices back into continuous joint vectors.
    if path_idx is not None:
        q_path = grid_path_to_q_path(path_idx, joint_limits, bins_per_joint)
        print(f"path waypoints:   {len(q_path)}")
        print("first waypoint:   ", q_path[0])
        print("last waypoint:    ", q_path[-1])

    checker.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
