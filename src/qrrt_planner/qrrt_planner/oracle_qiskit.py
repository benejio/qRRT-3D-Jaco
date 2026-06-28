"""
oracle_qiskit.py

Summary:
    Builds and runs Grover-style quantum circuits for qRRT candidate selection.

    The qRRT planner generates a local candidate set. A subset of those
    candidates is marked as "good", usually because they make better progress
    toward the goal. This module builds a Grover circuit that amplifies the
    probability of measuring those good candidate indices.

    The module can run circuits in two ways:
        - local simulation with Qiskit Aer
        - IBM Quantum backend through Qiskit Runtime, if enabled

    The output is a probability vector over candidate indices. qRRT then samples
    one candidate according to that probability distribution.

Important:
    This module does not decide which candidates are good. It only receives
    good_indices and builds the Grover-weighted probability distribution.

    Simulator wall-clock time should not be interpreted as real quantum hardware
    runtime. It is prototype execution time only.
"""

import math
import warnings
from typing import Iterable, Optional, Sequence

import numpy as np

from qiskit import QuantumCircuit, transpile
from qiskit_aer import AerSimulator

try:
    # Only needed when use_ibm=True.
    from qiskit_ibm_runtime import QiskitRuntimeService, SamplerV2 as Sampler
    _HAS_IBM_RUNTIME = True
except ImportError:
    _HAS_IBM_RUNTIME = False


def joint_goal_region_indices(candidates, q_goal, radius):
    """
    Find candidate joint vectors inside a Euclidean goal region.

    Args:
        candidates:
            Iterable or array of candidate joint vectors with shape (N, dof).
        q_goal:
            Goal joint vector with shape (dof,).
        radius:
            Euclidean joint-space radius threshold.

    Returns:
        List of integer candidate indices whose distance to q_goal is <= radius.
    """
    q_goal = np.asarray(q_goal, dtype=float)
    good = []

    for i, q in enumerate(candidates):
        q = np.asarray(q, dtype=float)
        if np.linalg.norm(q - q_goal) <= radius:
            good.append(i)

    return good


def joint_goal_region_indices_weighted(candidates, q_goal, per_joint_scales, radius):
    """
    Find candidate joint vectors inside a scaled/weighted goal region.

    This allows different joints to be weighted differently when measuring
    distance to the goal.

    Distance is computed as:
        sqrt(sum(((q_i - goal_i) / scale_i)^2))

    Args:
        candidates:
            Iterable or array of candidate joint vectors with shape (N, dof).
        q_goal:
            Goal joint vector with shape (dof,).
        per_joint_scales:
            Positive scale value for each joint.
        radius:
            Threshold in scaled distance.

    Returns:
        List of integer candidate indices inside the scaled goal region.
    """
    q_goal = np.asarray(q_goal, dtype=float)
    scales = np.asarray(per_joint_scales, dtype=float)
    good = []

    for i, q in enumerate(candidates):
        q = np.asarray(q, dtype=float)
        d = np.sqrt(np.sum(((q - q_goal) / scales) ** 2))
        if d <= radius:
            good.append(i)

    return good


def sample_index_from_probs(probs, rng=None):
    """
    Sample one integer index from a probability vector.

    If the probability vector is invalid or sums to zero, the function falls
    back to uniform sampling.

    Args:
        probs:
            1D probability or weight vector.
        rng:
            Optional NumPy random generator.

    Returns:
        Selected integer index.
    """
    probs = np.asarray(probs, dtype=float)

    if rng is None:
        rng = np.random.default_rng()

    if probs.ndim != 1 or len(probs) == 0:
        raise ValueError("probs must be a nonempty 1D array")

    total = float(np.sum(probs))
    if total <= 0.0 or not np.isfinite(total):
        probs = np.ones(len(probs), dtype=float) / float(len(probs))
    else:
        probs = probs / total

    return int(rng.choice(len(probs), p=probs))


def _apply_oracle_for_marked_states(qc, good_indices, n_qubits):
    """
    Apply a phase-flip oracle for all marked basis states.

    For each marked integer index:
        1. Convert the index to a bitstring.
        2. Use X gates to map that state to |11...1>.
        3. Apply a multi-controlled phase flip.
        4. Undo the X gates.

    Args:
        qc:
            QuantumCircuit modified in-place.
        good_indices:
            Integer basis indices to mark as good.
        n_qubits:
            Number of data qubits in the circuit.
    """
    if n_qubits == 1:
        for idx in good_indices:
            if idx == 1:
                qc.z(0)
            elif idx == 0:
                qc.x(0)
                qc.z(0)
                qc.x(0)
        return

    controls = list(range(n_qubits - 1))
    target = n_qubits - 1

    for idx in good_indices:
        bitstr = format(idx, f"0{n_qubits}b")

        # Qiskit uses little-endian qubit significance here:
        # qubit 0 is treated as the least-significant bit.
        zero_positions = []
        for q, bit in enumerate(reversed(bitstr)):
            if bit == "0":
                qc.x(q)
                zero_positions.append(q)

        # Multi-controlled Z implemented using H-MCX-H on the target qubit.
        qc.h(target)
        qc.mcx(controls, target)
        qc.h(target)

        # Uncompute the X gates.
        for q in zero_positions:
            qc.x(q)


def _apply_diffusion(qc, n_qubits):
    """
    Apply Grover's diffusion operator.

    The diffusion operator performs inversion about the mean, increasing the
    amplitudes of states marked by the oracle.
    """
    if n_qubits == 1:
        qc.h(0)
        qc.x(0)
        qc.z(0)
        qc.x(0)
        qc.h(0)
        return

    qc.h(range(n_qubits))
    qc.x(range(n_qubits))

    target = n_qubits - 1
    controls = list(range(n_qubits - 1))

    qc.h(target)
    qc.mcx(controls, target)
    qc.h(target)

    qc.x(range(n_qubits))
    qc.h(range(n_qubits))


def _build_grover_circuit(n_states, good_indices, n_iters):
    """
    Build a Grover circuit over n_states candidate indices.

    Args:
        n_states:
            Number of candidate states to represent.
        good_indices:
            Indices to amplify.
        n_iters:
            Number of Grover iterations.

    Returns:
        Tuple:
            qc:
                QuantumCircuit with measurements.
            n_qubits:
                Number of qubits used.

    Notes:
        The circuit uses ceil(log2(n_states)) qubits. If n_states is not a power
        of two, extra computational basis states exist but are ignored when
        probabilities are mapped back to candidate indices.
    """
    n_qubits = int(math.ceil(math.log2(max(2, n_states))))
    qc = QuantumCircuit(n_qubits)

    # Initialize uniform superposition.
    qc.h(range(n_qubits))

    # Apply Grover iterations.
    for _ in range(n_iters):
        _apply_oracle_for_marked_states(qc, good_indices, n_qubits)
        _apply_diffusion(qc, n_qubits)

    qc.measure_all()

    # Helpful for debugging candidate database size.
    print(f"Built Grover circuit with {qc.num_qubits} qubits")

    return qc, n_qubits


def _run_counts_aer(circuit, shots):
    """
    Run a measured quantum circuit on the local Aer simulator.

    Args:
        circuit:
            QuantumCircuit with measurements.
        shots:
            Number of samples.

    Returns:
        Measurement counts dictionary: bitstring -> count.
    """
    backend = AerSimulator()
    compiled = transpile(circuit, backend)
    job = backend.run(compiled, shots=shots)
    result = job.result()
    return result.get_counts()


def _meas_maps(transpiled_circuit):
    """
    Build qubit/classical-bit measurement maps after transpilation.

    Returns:
        q_to_c:
            q_to_c[q] = c means qubit q was measured into classical bit c.
        c_to_q:
            c_to_q[c] = q is the inverse map.

    This matters because IBM transpilation can reorder qubits and classical bits.
    """
    n_qubits = transpiled_circuit.num_qubits
    n_clbits = transpiled_circuit.num_clbits
    q_to_c = [None] * n_qubits
    c_to_q = [None] * n_clbits

    for inst, qargs, cargs in transpiled_circuit.data:
        if inst.name == "measure":
            q = transpiled_circuit.find_bit(qargs[0]).index
            c = transpiled_circuit.find_bit(cargs[0]).index
            q_to_c[q] = c
            c_to_q[c] = q

    return q_to_c, c_to_q


def _decode_count_bitstring(bitstr, c_to_q):
    """
    Convert a measured bitstring into the intended logical integer index.

    Args:
        bitstr:
            Qiskit count bitstring.
        c_to_q:
            Mapping from classical bits back to qubits.

    Returns:
        Logical candidate index.

    Notes:
        Qiskit count strings are ordered as c[n-1]...c[0].
        This function maps them back so qubit 0 is the least-significant bit.
    """
    n_clbits = len(bitstr)

    idx = 0
    for c in range(n_clbits):
        q = c_to_q[c]
        if q is None:
            continue

        bit_position = n_clbits - 1 - c
        b = 1 if bitstr[bit_position] == "1" else 0
        idx |= (b << q)

    return idx


def _run_counts_ibm(circuit, shots, backend_name, channel=None):
    """
    Run a measured quantum circuit on an IBM Quantum backend.

    Args:
        circuit:
            QuantumCircuit with measurements.
        shots:
            Number of backend shots.
        backend_name:
            IBM backend name.
        channel:
            Optional Qiskit Runtime channel.

    Returns:
        Tuple:
            counts:
                Measurement counts dictionary.
            isa_circuit:
                Transpiled circuit actually submitted to the backend.

    Raises:
        RuntimeError:
            If qiskit_ibm_runtime is not installed.
    """
    if not _HAS_IBM_RUNTIME:
        raise RuntimeError("qiskit_ibm_runtime is not installed in this env.")

    if channel is not None:
        service = QiskitRuntimeService(channel=channel)
    else:
        service = QiskitRuntimeService()

    backend = service.backend(backend_name)
    isa_circuit = transpile(circuit, backend=backend)

    sampler = Sampler(mode=backend)

    job = sampler.run([isa_circuit], shots=shots)
    result = job.result()

    pub_result = result[0]

    try:
        counts = pub_result.data.meas.get_counts()
    except AttributeError:
        counts = pub_result.data.get_counts()

    return counts, isa_circuit


def grover_weights_for_region(
    n_states,
    good_indices,
    n_iters=3,
    shots=2048,
    use_ibm=False,
    ibm_backend_name="ibmq_qasm_simulator",
    ibm_channel=None,
    return_counts=False,
):
    """
    Compute Grover-weighted probabilities over candidate indices.

    Args:
        n_states:
            Number of candidate states.
        good_indices:
            Candidate indices to mark and amplify.
        n_iters:
            Number of Grover iterations.
        shots:
            Number of measurement shots.
        use_ibm:
            If True, run on IBM Quantum. If False, run on Aer.
        ibm_backend_name:
            IBM backend name used when use_ibm=True.
        ibm_channel:
            Optional IBM Runtime channel.
        return_counts:
            If True, return raw counts and qubit count along with probabilities.

    Returns:
        If return_counts is False:
            probs:
                1D NumPy probability vector of length n_states.
        If return_counts is True:
            Tuple:
                probs:
                    Probability vector.
                counts:
                    Raw measurement counts.
                n_qubits:
                    Number of Grover qubits.

    Notes:
        If no good indices are provided, the function returns a uniform
        distribution.
    """
    good_indices = list(good_indices)

    if len(good_indices) == 0:
        probs = np.ones(n_states, dtype=float) / float(n_states)
        if return_counts:
            return probs, {}, int(math.ceil(math.log2(max(1, n_states))))
        return probs

    qc, n_qubits = _build_grover_circuit(n_states, good_indices, n_iters)

    if use_ibm:
        counts, isa_circuit = _run_counts_ibm(
            qc,
            shots=shots,
            backend_name=ibm_backend_name,
            channel=ibm_channel,
        )
        _, c_to_q = _meas_maps(isa_circuit)
    else:
        counts = _run_counts_aer(qc, shots=shots)
        c_to_q = None

    probs = np.zeros(n_states, dtype=float)
    total_shots = 0.0

    # Convert measured bitstrings into candidate-index probabilities.
    for bitstr, count in counts.items():
        if c_to_q is None:
            idx = int(bitstr, 2)
        else:
            idx = _decode_count_bitstring(bitstr, c_to_q)

        # Ignore unused basis states when n_states is not a power of two.
        if idx < n_states:
            probs[idx] += float(count)
            total_shots += float(count)

    if total_shots > 0.0:
        probs /= total_shots
    else:
        probs[:] = 1.0 / float(n_states)

    # Preserve the measured Grover distribution even when few shots land on
    # marked states. Low marked-shot counts are part of the quantum sampling
    # diagnostic and should not be replaced by uniform sampling.
    marked = set(good_indices)
    marked_shots = sum(
        probs[i] * total_shots for i in range(n_states) if i in marked
    )

    if marked_shots < 10:
        warnings.warn(
            f"Only ~{marked_shots:.0f} shots landed on marked states. "
            f"Using measured Grover distribution without uniform fallback.",
            RuntimeWarning,
        )

    if return_counts:
        return probs, counts, n_qubits
    return probs
