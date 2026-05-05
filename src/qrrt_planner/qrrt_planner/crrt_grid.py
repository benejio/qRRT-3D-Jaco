"""
crrt_grid.py

Summary:
    Implements Classical RRT on a discretized joint-space grid.

    Each grid index represents one possible robot joint configuration.
    The planner grows a tree from the start grid state toward randomly
    sampled target grid states. At each expansion, it finds the nearest
    existing tree node, builds a local set of valid neighboring candidates,
    scores those candidates by progress toward the goal, and chooses one
    candidate using a weighted classical selection rule.

    This planner is used as the main classical comparison against the
    quantum-guided grid-RRT planner.

Main concepts:
    GridIndex:
        A tuple of integer bin indices, one per robot joint.

    GridNode:
        A node in the RRT tree. Stores the grid index and parent pointer.

    local_candidate_set:
        Finds valid neighboring grid states near a tree node.

    choose_classical_candidate:
        Scores candidates by distance-to-goal and selects from the top-k.

    crrt_grid:
        Runs the full classical grid-RRT planning loop.
"""

from __future__ import annotations

import random
from dataclasses import dataclass
from typing import Callable, Dict, List, Optional, Sequence, Tuple

import numpy as np

from qrrt_planner.grid_joint_space import (
    GridIndex,
    grid_neighbors,
    make_grid_edge_checker,
    make_grid_validity_checker,
    nearest_valid_grid_index,
    q_to_grid,
)

# Type alias for a function that checks whether a grid state is valid.
GridStateValidFn = Callable[[GridIndex], bool]

# Type alias for a function that checks whether an edge between two grid states is valid.
GridEdgeValidFn = Callable[[GridIndex, GridIndex], bool]


@dataclass
class GridNode:
    """
    One node in the RRT tree.

    Attributes:
        idx:
            Grid index for this robot configuration.
        parent:
            Index of this node's parent in the nodes list.
            The root node has parent=None.
    """
    idx: GridIndex
    parent: Optional[int] = None


def idx_distance(a: GridIndex, b: GridIndex) -> float:
    """
    Compute Euclidean distance between two grid indices.

    This distance is measured in grid-index space, not physical workspace.
    """
    return float(np.linalg.norm(np.asarray(a, dtype=float) - np.asarray(b, dtype=float)))


def nearest_index_in_tree(nodes: List[GridNode], target: GridIndex) -> int:
    """
    Find the tree node closest to a target grid index.

    Args:
        nodes:
            Current RRT tree nodes.
        target:
            Target grid index.

    Returns:
        Integer index of the nearest node in the nodes list.
    """
    target_np = np.asarray(target, dtype=float)
    best_i = 0
    best_d = float(np.linalg.norm(np.asarray(nodes[0].idx, dtype=float) - target_np))

    for i in range(1, len(nodes)):
        d = float(np.linalg.norm(np.asarray(nodes[i].idx, dtype=float) - target_np))
        if d < best_d:
            best_d = d
            best_i = i

    return best_i


def reconstruct_grid_path(nodes: List[GridNode], goal_node_index: int) -> List[GridIndex]:
    """
    Reconstruct the final path by following parent links backward.

    Args:
        nodes:
            Full RRT tree.
        goal_node_index:
            Index of the final goal node.

    Returns:
        Ordered grid path from start to goal.
    """
    path: List[GridIndex] = []
    i = goal_node_index

    while i is not None:
        path.append(nodes[i].idx)
        i = nodes[i].parent

    path.reverse()
    return path


def grid_goal_reached(current: GridIndex, goal: GridIndex, goal_radius_idx: float) -> bool:
    """
    Check whether a grid index is close enough to the goal.

    Args:
        current:
            Current grid state.
        goal:
            Goal grid state.
        goal_radius_idx:
            Goal radius measured in grid-index distance.

    Returns:
        True if current is within goal_radius_idx of the goal.
    """
    return idx_distance(current, goal) <= goal_radius_idx


def sample_random_grid_index(shape: Sequence[int]) -> GridIndex:
    """
    Uniformly sample a random grid index.

    Args:
        shape:
            Number of bins per joint.

    Returns:
        Random grid index tuple.
    """
    return tuple(random.randrange(shape[d]) for d in range(len(shape)))


def local_candidate_set(
    center_idx: GridIndex,
    shape: Sequence[int],
    is_valid_idx: GridStateValidFn,
    edge_valid_idx: GridEdgeValidFn,
    max_candidates: int = 64,
    max_radius: int = 2,
) -> List[GridIndex]:
    """
    Build a local set of valid candidate states near a tree node.

    The search expands outward from center_idx using grid neighbors. A candidate
    is kept only if:
        1. the candidate grid state is valid,
        2. the edge from center_idx to the candidate is valid,
        3. the candidate has not already been added to this candidate set.

    Args:
        center_idx:
            Grid state around which local candidates are generated.
        shape:
            Grid shape, usually bins per joint.
        is_valid_idx:
            Function that checks whether a grid state is collision-free.
        edge_valid_idx:
            Function that checks whether an edge between grid states is valid.
        max_candidates:
            Maximum number of candidates to return.
        max_radius:
            Maximum number of neighbor-expansion layers.

    Returns:
        List of valid local candidate grid indices.
    """
    candidates: List[GridIndex] = []
    seen = set()

    frontier = [center_idx]
    visited = {center_idx}

    for _ in range(max_radius):
        new_frontier: List[GridIndex] = []
        for idx in frontier:
            for nbr in grid_neighbors(idx, shape):
                if nbr in visited:
                    continue
                visited.add(nbr)

                if not is_valid_idx(nbr):
                    continue
                if not edge_valid_idx(center_idx, nbr):
                    continue
                if nbr in seen:
                    continue

                seen.add(nbr)
                candidates.append(nbr)
                new_frontier.append(nbr)

                if len(candidates) >= max_candidates:
                    return candidates

        frontier = new_frontier
        if not frontier:
            break

    return candidates


def choose_classical_candidate(
    candidates: Sequence[GridIndex],
    goal_idx: GridIndex,
    rng: np.random.Generator,
    top_k: int = 8,
    progress_weight: float = 1.0,
) -> Tuple[GridIndex, int]:
    """
    Select one candidate using a classical goal-biased scoring rule.

    Candidates closer to the goal receive higher scores. The function sorts
    candidates by score, keeps the top_k candidates, and samples from them
    using softmax-like exponential weights.

    Args:
        candidates:
            Candidate grid states.
        goal_idx:
            Goal grid state.
        rng:
            NumPy random generator.
        top_k:
            Number of best candidates eligible for selection.
        progress_weight:
            Weight applied to distance-to-goal scoring.

    Returns:
        Tuple of:
            selected candidate grid index,
            rank of the selected candidate within the top-k set.
    """
    if not candidates:
        raise ValueError("No candidates provided to choose_classical_candidate")

    scored: List[Tuple[float, int]] = []
    for i, idx in enumerate(candidates):
        d_goal = idx_distance(idx, goal_idx)
        score = -progress_weight * d_goal
        scored.append((score, i))

    scored.sort(reverse=True)

    k = max(1, min(top_k, len(scored)))
    top = scored[:k]

    weights = np.array([np.exp(score) for score, _ in top], dtype=float)
    if float(np.sum(weights)) <= 0.0:
        probs = np.full(len(top), 1.0 / len(top), dtype=float)
    else:
        probs = weights / np.sum(weights)

    chosen_local = int(rng.choice(len(top), p=probs))
    chosen_idx = top[chosen_local][1]
    return candidates[chosen_idx], chosen_local + 1


def crrt_grid(
    q_start: Sequence[float],
    q_goal: Sequence[float],
    bins: Sequence[np.ndarray],
    state_valid_fn,
    max_iters: int = 5000,
    goal_sample_rate: float = 0.10,
    goal_radius_idx: float = 0.0,
    edge_step: float = 0.05,
    candidate_count: int = 64,
    top_k: int = 8,
    progress_weight: float = 1.0,
    rng_seed: int = 0,
    debug: bool = False,
):
    """
    Run classical RRT over a discretized joint-space grid.

    Args:
        q_start:
            Continuous start joint configuration.
        q_goal:
            Continuous goal joint configuration.
        bins:
            Discretized joint values for each joint.
        state_valid_fn:
            Function that checks whether a continuous joint state is valid.
        max_iters:
            Maximum number of RRT iterations.
        goal_sample_rate:
            Probability of sampling the goal instead of a random grid state.
        goal_radius_idx:
            Goal radius in grid-index distance.
        edge_step:
            Step size used for edge validity checking.
        candidate_count:
            Maximum number of local candidates considered per expansion.
        top_k:
            Number of best-scoring candidates used for selection.
        progress_weight:
            Weight for goal-progress scoring.
        rng_seed:
            Random seed for reproducible runs.
        debug:
            If True, store per-iteration trace information.

    Returns:
        Tuple:
            path:
                List of grid indices from start to goal, or None on failure.
            stats:
                Dictionary of planner metrics.
    """
    rng = np.random.default_rng(rng_seed)
    random.seed(rng_seed)

    shape = tuple(len(b) for b in bins)
    start_idx_raw = q_to_grid(q_start, bins)
    goal_idx_raw = q_to_grid(q_goal, bins)

    # Create grid-level validity and edge-checking functions from the continuous
    # MoveIt collision checker.
    is_valid_idx = make_grid_validity_checker(bins, state_valid_fn)
    edge_valid_idx = make_grid_edge_checker(bins, state_valid_fn, edge_step)

    # Snap start and goal to nearby valid grid states.
    start_idx = nearest_valid_grid_index(start_idx_raw, shape, is_valid_idx, max_radius=3)
    goal_idx = nearest_valid_grid_index(goal_idx_raw, shape, is_valid_idx, max_radius=3)

    if start_idx is None:
        raise ValueError("No valid grid state found near start")
    if goal_idx is None:
        raise ValueError("No valid grid state found near goal")

    # Initialize RRT tree with the start node.
    nodes: List[GridNode] = [GridNode(idx=start_idx, parent=None)]
    visited_tree = {start_idx}

    # Metrics collected for reporting and CSV output.
    stats: Dict[str, object] = {
        "planner_type": "crrt_grid",
        "success": False,
        "iterations": 0,
        "nodes": 1,
        "start_idx_raw": start_idx_raw,
        "goal_idx_raw": goal_idx_raw,
        "start_idx": start_idx,
        "goal_idx": goal_idx,
        "classical_calls": 0,
        "classical_successful_extensions": 0,
        "invalid_random_targets": 0,
        "empty_candidate_sets": 0,
        "goal_sample_hits": 0,
        "duplicate_rejections": 0,
        "invalid_chosen_state_rejections": 0,
        "invalid_chosen_edge_rejections": 0,
        "candidate_count_total": 0,
        "candidate_count_nonempty_iters": 0,
        "selected_rank_total": 0,
        "path_waypoints": None,
        "trace": [] if debug else None,
    }

    for it in range(max_iters):
        stats["iterations"] = it + 1

        # With probability goal_sample_rate, sample the goal directly.
        # Otherwise sample a random grid state.
        goal_sampled = random.random() < goal_sample_rate

        if goal_sampled:
            q_rand_idx = goal_idx
            stats["goal_sample_hits"] = int(stats["goal_sample_hits"]) + 1
        else:
            q_rand_idx = sample_random_grid_index(shape)

        # Reject sampled targets that are invalid.
        if not is_valid_idx(q_rand_idx):
            stats["invalid_random_targets"] = int(stats["invalid_random_targets"]) + 1
            if debug:
                stats["trace"].append({
                    "iteration": it + 1,
                    "goal_sampled": goal_sampled,
                    "candidate_count": 0,
                    "chosen_idx": None,
                    "chosen_rank": None,
                    "accepted": False,
                    "rejection_reason": "invalid_random_target",
                    "tree_nodes": len(nodes),
                })
            continue

        # Find nearest existing tree node to the sampled target.
        i_near = nearest_index_in_tree(nodes, q_rand_idx)
        idx_near = nodes[i_near].idx

        # Build valid local candidate states around the nearest tree node.
        candidates = local_candidate_set(
            center_idx=idx_near,
            shape=shape,
            is_valid_idx=is_valid_idx,
            edge_valid_idx=edge_valid_idx,
            max_candidates=candidate_count,
            max_radius=2,
        )
        stats["candidate_count_total"] = int(stats["candidate_count_total"]) + len(candidates)

        if not candidates:
            stats["empty_candidate_sets"] = int(stats["empty_candidate_sets"]) + 1
            if debug:
                stats["trace"].append({
                    "iteration": it + 1,
                    "goal_sampled": goal_sampled,
                    "candidate_count": 0,
                    "chosen_idx": None,
                    "chosen_rank": None,
                    "accepted": False,
                    "rejection_reason": "empty_candidate_set",
                    "tree_nodes": len(nodes),
                })
            continue

        stats["candidate_count_nonempty_iters"] = int(stats["candidate_count_nonempty_iters"]) + 1
        stats["classical_calls"] = int(stats["classical_calls"]) + 1

        # Select one candidate using the classical goal-progress rule.
        idx_new, chosen_rank = choose_classical_candidate(
            candidates=candidates,
            goal_idx=goal_idx,
            rng=rng,
            top_k=top_k,
            progress_weight=progress_weight,
        )
        stats["selected_rank_total"] = int(stats["selected_rank_total"]) + chosen_rank

        rejection_reason = None
        accepted = False

        # Validate and add the selected candidate to the tree.
        if idx_new in visited_tree:
            stats["duplicate_rejections"] = int(stats["duplicate_rejections"]) + 1
            rejection_reason = "duplicate"
        elif not is_valid_idx(idx_new):
            stats["invalid_chosen_state_rejections"] = int(stats["invalid_chosen_state_rejections"]) + 1
            rejection_reason = "invalid_chosen_state"
        elif not edge_valid_idx(idx_near, idx_new):
            stats["invalid_chosen_edge_rejections"] = int(stats["invalid_chosen_edge_rejections"]) + 1
            rejection_reason = "invalid_chosen_edge"
        else:
            nodes.append(GridNode(idx=idx_new, parent=i_near))
            visited_tree.add(idx_new)
            stats["nodes"] = len(nodes)
            stats["classical_successful_extensions"] = int(stats["classical_successful_extensions"]) + 1
            accepted = True

            # If the new node reaches the goal and the final edge is valid,
            # append the goal and return the path.
            if grid_goal_reached(idx_new, goal_idx, goal_radius_idx):
                if edge_valid_idx(idx_new, goal_idx):
                    nodes.append(GridNode(idx=goal_idx, parent=len(nodes) - 1))
                    stats["nodes"] = len(nodes)
                    stats["success"] = True
                    path = reconstruct_grid_path(nodes, len(nodes) - 1)
                    stats["path_waypoints"] = len(path)

                    if debug:
                        stats["trace"].append({
                            "iteration": it + 1,
                            "goal_sampled": goal_sampled,
                            "candidate_count": len(candidates),
                            "chosen_idx": idx_new,
                            "chosen_rank": chosen_rank,
                            "accepted": True,
                            "rejection_reason": None,
                            "tree_nodes": len(nodes),
                        })

                    return path, stats

        if debug:
            stats["trace"].append({
                "iteration": it + 1,
                "goal_sampled": goal_sampled,
                "candidate_count": len(candidates),
                "chosen_idx": idx_new,
                "chosen_rank": chosen_rank,
                "accepted": accepted,
                "rejection_reason": rejection_reason,
                "tree_nodes": len(nodes),
            })

    # If max_iters is reached without reaching the goal, report failure.
    return None, stats
