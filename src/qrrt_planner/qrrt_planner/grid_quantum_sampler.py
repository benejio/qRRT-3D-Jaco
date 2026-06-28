"""
grid_quantum_sampler.py

Summary:
    Provides the Grover-based unordered candidate-selection step used by qRRT.

    The planner first builds a local candidate database using classical ROS 2 /
    MoveIt checks. That database is intentionally treated as unordered: the
    candidate order is randomized before it is encoded as computational basis
    states.

    This module does not sort candidates. Instead, it defines an oracle-style
    Boolean predicate over candidate indices. The predicate marks candidates
    whose local extension score is better than the current incumbent score.
    Repeated Grover searches then perform a simple quantum best-candidate search:
    Grover amplifies candidates that beat the incumbent, measurement proposes a
    better incumbent, and the process repeats for a fixed number of rounds.

    In simulation, the marked basis indices must still be computed classically
    so Qiskit can construct the oracle. Algorithmically, those indices represent
    the states for which the oracle predicate evaluates to true.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Sequence, Tuple

import numpy as np

from qrrt_planner.oracle_qiskit import (
    grover_weights_for_region,
    sample_index_from_probs,
)

# A grid index is a tuple of integer bin positions, one per robot joint.
# Example for a 6-DOF arm:
#     (5, 5, 7, 5, 2, 7)
GridIndex = Tuple[int, ...]


@dataclass(frozen=True)
class QuantumSelectionResult:
    """
    Result of one unordered Grover best-candidate selection.

    Attributes:
        candidate:
            Selected grid candidate.
        candidate_position:
            Position of the selected candidate in the unordered candidate list.
        selected_score:
            Local extension score of the selected candidate.
        initial_position:
            Random initial incumbent position.
        initial_score:
            Score of the random initial incumbent.
        marked_count_total:
            Total number of marked states across Grover-improvement rounds.
        grover_rounds:
            Number of Grover circuits/searches actually executed.
        improved:
            True if Grover measurement improved on the initial incumbent.
    """

    candidate: GridIndex
    candidate_position: int
    selected_score: float
    initial_position: int
    initial_score: float
    marked_count_total: int
    grover_rounds: int
    improved: bool


def grid_idx_distance(a: GridIndex, b: GridIndex) -> float:
    """
    Compute Euclidean distance between two grid indices.

    This distance is measured in grid-index space, not physical workspace.
    """
    return float(np.linalg.norm(np.asarray(a, dtype=float) - np.asarray(b, dtype=float)))


def candidate_extension_score(
    candidate_idx: GridIndex,
    near_idx: GridIndex,
    target_idx: GridIndex,
    goal_idx: GridIndex,
    target_weight: float = 1.0,
    goal_weight: float = 1.0,
) -> float:
    """
    Score a local tree-extension candidate without sorting the database.

    Larger scores are better. The score rewards candidates that move away from
    q_near toward the current RRT target q_rand and, optionally, toward the goal.

    A positive target-progress term means:
        dist(q_near, q_rand) - dist(candidate, q_rand) > 0

    A positive goal-progress term means:
        dist(q_near, q_goal) - dist(candidate, q_goal) > 0
    """
    target_progress = grid_idx_distance(near_idx, target_idx) - grid_idx_distance(
        candidate_idx,
        target_idx,
    )
    goal_progress = grid_idx_distance(near_idx, goal_idx) - grid_idx_distance(
        candidate_idx,
        goal_idx,
    )

    return float(target_weight * target_progress + goal_weight * goal_progress)


def marked_indices_better_than_threshold(
    candidates: Sequence[GridIndex],
    near_idx: GridIndex,
    target_idx: GridIndex,
    goal_idx: GridIndex,
    score_threshold: float,
    target_weight: float = 1.0,
    goal_weight: float = 1.0,
    score_margin: float = 1e-9,
) -> List[int]:
    """
    Return unordered candidate positions whose score beats a threshold.

    This is intentionally not a ranking operation. It is the simulator-side
    equivalent of evaluating a Boolean oracle predicate:

        f(i) = 1 if score(candidate_i) > score_threshold + score_margin
        f(i) = 0 otherwise
    """
    marked: List[int] = []

    for i, candidate_idx in enumerate(candidates):
        score = candidate_extension_score(
            candidate_idx=candidate_idx,
            near_idx=near_idx,
            target_idx=target_idx,
            goal_idx=goal_idx,
            target_weight=target_weight,
            goal_weight=goal_weight,
        )
        if score > score_threshold + score_margin:
            marked.append(i)

    return marked


def quantum_select_grid_candidate(
    candidates: Sequence[GridIndex],
    near_idx: GridIndex,
    target_idx: GridIndex,
    goal_idx: GridIndex,
    rng: np.random.Generator,
    n_iters: int = 1,
    shots: int = 64,
    use_ibm: bool = False,
    ibm_backend_name: str = "ibm_torino",
    ibm_channel: str | None = None,
    target_weight: float = 1.0,
    goal_weight: float = 1.0,
    best_rounds: int = 3,
    score_margin: float = 1e-9,
) -> QuantumSelectionResult:
    """
    Select one grid candidate using unordered Grover improvement search.

    Steps:
        1. Start from a random incumbent candidate in the unordered database.
        2. Mark all candidates whose oracle score is better than the incumbent.
        3. Use Grover amplification and measurement to sample a better candidate.
        4. If the measured candidate is better, make it the new incumbent.
        5. Repeat for best_rounds or until no better candidate exists.

    This avoids the previous classical top-k sorting step. Collision checking
    and candidate usability are still handled before this function is called.
    """
    if not candidates:
        raise ValueError("No candidates provided to quantum_select_grid_candidate")

    if best_rounds < 1:
        best_rounds = 1

    # Random initial incumbent. The candidate database is already shuffled by
    # the planner, but this also avoids selecting index 0 by convention.
    incumbent_pos = int(rng.integers(0, len(candidates)))
    incumbent = candidates[incumbent_pos]
    incumbent_score = candidate_extension_score(
        candidate_idx=incumbent,
        near_idx=near_idx,
        target_idx=target_idx,
        goal_idx=goal_idx,
        target_weight=target_weight,
        goal_weight=goal_weight,
    )

    initial_pos = incumbent_pos
    initial_score = incumbent_score
    marked_count_total = 0
    grover_rounds = 0
    improved = False

    for _ in range(best_rounds):
        marked_indices = marked_indices_better_than_threshold(
            candidates=candidates,
            near_idx=near_idx,
            target_idx=target_idx,
            goal_idx=goal_idx,
            score_threshold=incumbent_score,
            target_weight=target_weight,
            goal_weight=goal_weight,
            score_margin=score_margin,
        )
        marked_count_total += len(marked_indices)

        if not marked_indices:
            break

        probs = grover_weights_for_region(
            n_states=len(candidates),
            good_indices=marked_indices,
            n_iters=n_iters,
            shots=shots,
            use_ibm=use_ibm,
            ibm_backend_name=ibm_backend_name,
            ibm_channel=ibm_channel,
            return_counts=False,
        )
        grover_rounds += 1

        measured_pos = int(sample_index_from_probs(probs, rng=rng))
        measured = candidates[measured_pos]
        measured_score = candidate_extension_score(
            candidate_idx=measured,
            near_idx=near_idx,
            target_idx=target_idx,
            goal_idx=goal_idx,
            target_weight=target_weight,
            goal_weight=goal_weight,
        )

        # Noisy hardware or unlucky measurement can still return an unmarked or
        # worse state. Only accept a true improvement.
        if measured_score > incumbent_score + score_margin:
            incumbent_pos = measured_pos
            incumbent = measured
            incumbent_score = measured_score
            improved = True
        else:
            break

    return QuantumSelectionResult(
        candidate=incumbent,
        candidate_position=incumbent_pos,
        selected_score=float(incumbent_score),
        initial_position=initial_pos,
        initial_score=float(initial_score),
        marked_count_total=int(marked_count_total),
        grover_rounds=int(grover_rounds),
        improved=bool(improved),
    )
