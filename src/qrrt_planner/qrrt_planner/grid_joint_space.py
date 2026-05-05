"""
grid_joint_space.py

Summary:
    Defines the discretized joint-space grid used by the planning experiments.

    The robot's continuous 6-DOF joint space is converted into a finite grid.
    Each grid index represents one possible robot joint configuration. This
    module provides helper functions for:

        - creating joint bins from joint limits,
        - converting between continuous joint vectors and grid indices,
        - finding neighboring grid cells,
        - checking state and edge validity through MoveIt,
        - finding nearby valid grid states,
        - running A* search over the grid,
        - converting grid paths back into continuous joint-space paths.

    This file is shared by the A*, classical grid-RRT, and quantum grid-RRT
    planners.
"""

from __future__ import annotations

import heapq
import math
from functools import lru_cache
from typing import Callable, Dict, List, Optional, Sequence, Tuple

import numpy as np


ArrayLike = Sequence[float]
GridIndex = Tuple[int, ...]
StateValidFn = Callable[[np.ndarray], bool]


def make_joint_bins(
    joint_limits: Sequence[Tuple[float, float]],
    bins_per_joint: Sequence[int],
) -> List[np.ndarray]:
    """
    Create evenly spaced joint values for each joint.

    Args:
        joint_limits:
            List of (lower, upper) limits for each joint.
        bins_per_joint:
            Number of discrete grid values for each joint.

    Returns:
        List of NumPy arrays. Each array contains the allowed grid values
        for one joint.
    """
    if len(joint_limits) != len(bins_per_joint):
        raise ValueError("joint_limits and bins_per_joint must have the same length")

    bins: List[np.ndarray] = []
    for (lo, hi), n_bins in zip(joint_limits, bins_per_joint):
        if n_bins < 2:
            raise ValueError("Each joint must have at least 2 bins")
        bins.append(np.linspace(lo, hi, n_bins, dtype=float))
    return bins


def grid_shape_from_bins(bins: Sequence[np.ndarray]) -> Tuple[int, ...]:
    """
    Return the grid shape from the bin arrays.

    Example:
        Six joints with 10 bins each gives:
            (10, 10, 10, 10, 10, 10)
    """
    return tuple(len(b) for b in bins)


def grid_to_q(idx: GridIndex, bins: Sequence[np.ndarray]) -> np.ndarray:
    """
    Convert a grid index into a continuous joint vector.

    Args:
        idx:
            Grid index, such as (5, 5, 7, 5, 2, 7).
        bins:
            Joint bin arrays.

    Returns:
        Continuous joint vector q.
    """
    if len(idx) != len(bins):
        raise ValueError("Grid index dimension does not match number of joints")

    q = np.array([bins[d][idx[d]] for d in range(len(idx))], dtype=float)
    return q


def q_to_grid(q: ArrayLike, bins: Sequence[np.ndarray]) -> GridIndex:
    """
    Snap a continuous joint vector to the nearest grid index.

    Args:
        q:
            Continuous joint vector.
        bins:
            Joint bin arrays.

    Returns:
        Nearest grid index.
    """
    q_np = np.asarray(q, dtype=float)
    if len(q_np) != len(bins):
        raise ValueError("Joint vector dimension does not match number of joints")

    idx: List[int] = []
    for d, qd in enumerate(q_np):
        arr = bins[d]
        i = int(np.argmin(np.abs(arr - qd)))
        idx.append(i)
    return tuple(idx)


def grid_neighbors(
    idx: GridIndex,
    shape: Sequence[int],
) -> List[GridIndex]:
    """
    Return axis-aligned neighboring grid cells.

    In a 6-DOF grid, each state can have up to 12 neighbors:
        - one step down/up in each of the 6 joint dimensions.

    Args:
        idx:
            Current grid index.
        shape:
            Grid dimensions.

    Returns:
        List of neighboring grid indices within bounds.
    """
    if len(idx) != len(shape):
        raise ValueError("idx and shape must have the same dimension")

    nbrs: List[GridIndex] = []
    for d in range(len(idx)):
        if idx[d] - 1 >= 0:
            left = list(idx)
            left[d] -= 1
            nbrs.append(tuple(left))

        if idx[d] + 1 < shape[d]:
            right = list(idx)
            right[d] += 1
            nbrs.append(tuple(right))

    return nbrs


def dist_idx(a: GridIndex, b: GridIndex) -> float:
    """
    Compute Euclidean distance between two grid indices.
    """
    return math.sqrt(sum((ai - bi) ** 2 for ai, bi in zip(a, b)))


def dist_q(q1: np.ndarray, q2: np.ndarray) -> float:
    """
    Compute Euclidean distance between two continuous joint vectors.
    """
    return float(np.linalg.norm(q1 - q2))


def interpolate_edge_q(
    q_from: np.ndarray,
    q_to: np.ndarray,
    edge_step: float,
) -> List[np.ndarray]:
    """
    Interpolate intermediate joint states along a straight-line joint-space edge.

    These intermediate states are used for collision checking an edge between
    two configurations.

    Args:
        q_from:
            Start joint vector.
        q_to:
            End joint vector.
        edge_step:
            Approximate spacing between interpolated states.

    Returns:
        List of intermediate joint vectors, including q_to but not q_from.
    """
    d = np.linalg.norm(q_to - q_from)
    n_steps = max(2, int(np.ceil(d / edge_step)))

    qs: List[np.ndarray] = []
    for i in range(1, n_steps + 1):
        alpha = float(i) / float(n_steps)
        q_interp = (1.0 - alpha) * q_from + alpha * q_to
        qs.append(q_interp)

    return qs


def edge_is_valid_q(
    q_from: np.ndarray,
    q_to: np.ndarray,
    state_valid_fn: StateValidFn,
    edge_step: float,
) -> bool:
    """
    Check whether a continuous joint-space edge is valid.

    The edge is valid only if every interpolated state along the edge is valid.

    Args:
        q_from:
            Start joint vector.
        q_to:
            End joint vector.
        state_valid_fn:
            Function that checks if a continuous joint state is collision-free.
        edge_step:
            Interpolation step size.

    Returns:
        True if the full edge is valid, otherwise False.
    """
    for q in interpolate_edge_q(q_from, q_to, edge_step):
        if not state_valid_fn(q):
            return False
    return True


def make_grid_validity_checker(
    bins: Sequence[np.ndarray],
    state_valid_fn: StateValidFn,
) -> Callable[[GridIndex], bool]:
    """
    Create a cached validity checker for grid states.

    The returned function converts a grid index to a joint vector and checks it
    with MoveIt. Results are cached so repeated checks of the same grid cell are
    fast.
    """
    @lru_cache(maxsize=None)
    def is_valid(idx: GridIndex) -> bool:
        q = grid_to_q(idx, bins)
        return bool(state_valid_fn(q))

    return is_valid


def make_grid_edge_checker(
    bins: Sequence[np.ndarray],
    state_valid_fn: StateValidFn,
    edge_step: float,
) -> Callable[[GridIndex, GridIndex], bool]:
    """
    Create a cached edge checker for grid-state pairs.

    The returned function converts two grid indices to continuous joint vectors
    and checks the interpolated edge between them.
    """
    @lru_cache(maxsize=None)
    def edge_valid(idx_a: GridIndex, idx_b: GridIndex) -> bool:
        q_a = grid_to_q(idx_a, bins)
        q_b = grid_to_q(idx_b, bins)
        return edge_is_valid_q(q_a, q_b, state_valid_fn, edge_step)

    return edge_valid


def reconstruct_path(
    came_from: Dict[GridIndex, GridIndex],
    current: GridIndex,
) -> List[GridIndex]:
    """
    Reconstruct an A* path by following parent links backward.
    """
    path = [current]
    while current in came_from:
        current = came_from[current]
        path.append(current)
    path.reverse()
    return path


def in_bounds(idx: GridIndex, shape: Sequence[int]) -> bool:
    """
    Check whether a grid index is inside the grid bounds.
    """
    return all(0 <= idx[d] < shape[d] for d in range(len(idx)))


def nearby_indices_linf(center: GridIndex, shape: Sequence[int], radius: int) -> List[GridIndex]:
    """
    Return all grid indices within an L-infinity cube around a center index.

    This is used to search nearby cells when the snapped start or goal cell is
    invalid. For example, radius=1 checks all cells within one bin in every
    joint dimension.
    """
    if radius < 0:
        return []

    results: List[GridIndex] = []
    dims = len(center)

    def rec_build(d: int, current: List[int]) -> None:
        if d == dims:
            idx = tuple(current)
            if in_bounds(idx, shape):
                results.append(idx)
            return

        lo = center[d] - radius
        hi = center[d] + radius
        for val in range(lo, hi + 1):
            current.append(val)
            rec_build(d + 1, current)
            current.pop()

    rec_build(0, [])
    return results


def nearest_valid_grid_index(
    target_idx: GridIndex,
    shape: Sequence[int],
    is_valid_fn,
    max_radius: int = 3,
) -> Optional[GridIndex]:
    """
    Search outward from a target grid index for the nearest valid grid cell.

    This is useful because a continuous start or goal may snap to an invalid
    grid cell. The planner can instead use a nearby valid grid cell.

    Args:
        target_idx:
            Original snapped grid index.
        shape:
            Grid dimensions.
        is_valid_fn:
            Grid validity checker.
        max_radius:
            Maximum L-infinity search radius.

    Returns:
        Nearest valid grid index, or None if no valid index is found.
    """
    if is_valid_fn(target_idx):
        return target_idx

    best_idx: Optional[GridIndex] = None
    best_dist = float("inf")

    for radius in range(1, max_radius + 1):
        candidates = nearby_indices_linf(target_idx, shape, radius)
        for idx in candidates:
            if not is_valid_fn(idx):
                continue
            d = dist_idx(idx, target_idx)
            if d < best_dist:
                best_dist = d
                best_idx = idx

        if best_idx is not None:
            return best_idx

    return None


def astar_grid_joint_space(
    q_start: ArrayLike,
    q_goal: ArrayLike,
    joint_limits: Sequence[Tuple[float, float]],
    bins_per_joint: Sequence[int],
    state_valid_fn: StateValidFn,
    edge_step: float = 0.05,
) -> Tuple[Optional[List[GridIndex]], Dict[str, object]]:
    """
    Run A* search over the discretized joint-space grid.

    Args:
        q_start:
            Continuous start joint vector.
        q_goal:
            Continuous goal joint vector.
        joint_limits:
            Joint limits for each joint.
        bins_per_joint:
            Number of discretization bins per joint.
        state_valid_fn:
            Function that checks continuous joint-state validity.
        edge_step:
            Interpolation step size for edge collision checking.

    Returns:
        path_idx:
            List of grid indices from start to goal, or None on failure.
        stats:
            Dictionary containing search metrics.
    """
    bins = make_joint_bins(joint_limits, bins_per_joint)
    shape = grid_shape_from_bins(bins)

    start_idx_raw = q_to_grid(q_start, bins)
    goal_idx_raw = q_to_grid(q_goal, bins)

    is_valid = make_grid_validity_checker(bins, state_valid_fn)
    edge_valid = make_grid_edge_checker(bins, state_valid_fn, edge_step)

    start_idx = nearest_valid_grid_index(start_idx_raw, shape, is_valid, max_radius=3)
    goal_idx = nearest_valid_grid_index(goal_idx_raw, shape, is_valid, max_radius=3)

    if start_idx is None:
        raise ValueError("No valid grid state found near start")
    if goal_idx is None:
        raise ValueError("No valid grid state found near goal")

    q_start_snap = grid_to_q(start_idx, bins)
    q_goal_snap = grid_to_q(goal_idx, bins)

    stats: Dict[str, object] = {
        "start_idx_raw": start_idx_raw,
        "goal_idx_raw": goal_idx_raw,
        "start_idx": start_idx,
        "goal_idx": goal_idx,
        "start_q_snapped": q_start_snap,
        "goal_q_snapped": q_goal_snap,
        "expanded_nodes": 0,
        "visited_nodes": 0,
        "invalid_neighbor_states": 0,
        "invalid_neighbor_edges": 0,
        "path_length_idx": None,
        "path_length_q": None,
        "success": False,
    }

    # Priority queue entries are:
    #     (f_score, g_score, grid_index)
    # where f_score = path cost so far + heuristic distance to goal.
    open_heap: List[Tuple[float, float, GridIndex]] = []
    heapq.heappush(open_heap, (dist_idx(start_idx, goal_idx), 0.0, start_idx))

    came_from: Dict[GridIndex, GridIndex] = {}
    g_score: Dict[GridIndex, float] = {start_idx: 0.0}
    closed: set[GridIndex] = set()
    discovered: set[GridIndex] = {start_idx}

    while open_heap:
        _, current_g, current = heapq.heappop(open_heap)

        if current in closed:
            continue

        closed.add(current)
        stats["expanded_nodes"] = int(stats["expanded_nodes"]) + 1

        # Goal reached: reconstruct path and compute path metrics.
        if current == goal_idx:
            path_idx = reconstruct_path(came_from, current)
            path_q = [grid_to_q(idx, bins) for idx in path_idx]

            stats["visited_nodes"] = len(discovered)
            stats["path_length_idx"] = len(path_idx) - 1
            stats["path_length_q"] = sum(
                dist_q(path_q[i - 1], path_q[i]) for i in range(1, len(path_q))
            )
            stats["success"] = True
            return path_idx, stats

        # Expand neighboring grid cells.
        for nbr in grid_neighbors(current, shape):
            if nbr in closed:
                continue

            discovered.add(nbr)

            if not is_valid(nbr):
                stats["invalid_neighbor_states"] = int(stats["invalid_neighbor_states"]) + 1
                continue

            if not edge_valid(current, nbr):
                stats["invalid_neighbor_edges"] = int(stats["invalid_neighbor_edges"]) + 1
                continue

            tentative_g = current_g + 1.0

            if tentative_g < g_score.get(nbr, float("inf")):
                g_score[nbr] = tentative_g
                came_from[nbr] = current
                f_score = tentative_g + dist_idx(nbr, goal_idx)
                heapq.heappush(open_heap, (f_score, tentative_g, nbr))

    # If the queue empties, no path was found.
    stats["visited_nodes"] = len(discovered)
    return None, stats


def grid_path_to_q_path(
    path_idx: Sequence[GridIndex],
    joint_limits: Sequence[Tuple[float, float]],
    bins_per_joint: Sequence[int],
) -> List[np.ndarray]:
    """
    Convert a path of grid indices into continuous joint vectors.
    """
    bins = make_joint_bins(joint_limits, bins_per_joint)
    return [grid_to_q(idx, bins) for idx in path_idx]


def total_grid_states(bins_per_joint: Sequence[int]) -> int:
    """
    Compute the total number of states in the discretized grid.

    Example:
        6 joints with 10 bins each gives 10^6 = 1,000,000 states.
    """
    total = 1
    for n in bins_per_joint:
        total *= int(n)
    return total


if __name__ == "__main__":
    print("This module defines a discretized joint-space grid and A* search.")
