from __future__ import annotations

from typing import Callable, List, Sequence, Tuple


def balance_tolerance(lengths: Sequence[float]) -> float:
    if not lengths:
        return 0.0
    mean = sum(lengths) / len(lengths)
    return 0.01 * mean


def is_balanced(lengths: Sequence[float]) -> bool:
    if not lengths:
        return True
    tol = balance_tolerance(lengths)
    return max(lengths) - min(lengths) <= tol


def initial_cuts(num_strips: int, num_groups: int) -> List[int]:
    if num_groups <= 0:
        raise ValueError("num_groups must be positive")
    cuts = [0]
    if num_strips == 0:
        return [0] * (num_groups + 1)
    base = num_strips // num_groups
    remainder = num_strips % num_groups
    pos = 0
    for index in range(num_groups):
        size = base + (1 if index < remainder else 0)
        pos += size
        cuts.append(pos)
    return cuts


def slices_from_cuts(cuts: Sequence[int]) -> List[slice]:
    return [slice(cuts[index], cuts[index + 1]) for index in range(len(cuts) - 1)]


def validate_cuts(cuts: Sequence[int], num_strips: int, num_groups: int) -> bool:
    if len(cuts) != num_groups + 1:
        return False
    if cuts[0] != 0 or cuts[-1] != num_strips:
        return False
    return all(cuts[index] < cuts[index + 1] for index in range(len(cuts) - 1))


def balance_slices(
    num_strips: int,
    num_groups: int,
    length_fn: Callable[[slice], float] | None = None,
    lengths_fn: Callable[[Sequence[int]], List[float]] | None = None,
    max_iterations: int = 50,
) -> Tuple[List[slice], List[float]]:
    if length_fn is None and lengths_fn is None:
        raise ValueError("provide length_fn or lengths_fn")

    cuts = initial_cuts(num_strips, num_groups)
    if not validate_cuts(cuts, num_strips, num_groups):
        empty = [slice(0, 0) for _ in range(num_groups)]
        return empty, [0.0] * num_groups

    def lengths_for_cuts(current: Sequence[int]) -> List[float]:
        if lengths_fn is not None:
            return lengths_fn(current)
        return [length_fn(slice(current[i], current[i + 1])) for i in range(num_groups)]  # type: ignore[misc]

    lengths = lengths_for_cuts(cuts)
    best_cuts = list(cuts)
    best_lengths = list(lengths)
    best_max = max(lengths)

    for _ in range(max_iterations):
        if is_balanced(lengths):
            break

        max_index = lengths.index(max(lengths))
        min_index = lengths.index(min(lengths))
        improved = False
        candidates: List[List[int]] = []

        if max_index != min_index:
            if min_index < max_index and cuts[max_index] < cuts[max_index + 1] - 1:
                trial = list(cuts)
                trial[max_index] += 1
                if validate_cuts(trial, num_strips, num_groups):
                    candidates.append(trial)
            if min_index > max_index and cuts[max_index] < cuts[max_index + 1] - 1:
                trial = list(cuts)
                trial[max_index + 1] -= 1
                if validate_cuts(trial, num_strips, num_groups):
                    candidates.append(trial)

        for trial in candidates:
            trial_lengths = lengths_for_cuts(trial)
            if max(trial_lengths) < max(lengths):
                cuts = trial
                lengths = trial_lengths
                improved = True
                break

        current_max = max(lengths)
        if current_max < best_max:
            best_max = current_max
            best_cuts = list(cuts)
            best_lengths = list(lengths)

        if not improved:
            break

    return slices_from_cuts(best_cuts), best_lengths


def balance_circle_boundaries(
    boundaries: List[float],
    length_fn: Callable[[List[float]], List[float]],
    max_iterations: int = 50,
) -> Tuple[List[float], List[float]]:
    angles = list(boundaries)
    lengths = length_fn(angles)
    best_angles = list(angles)
    best_lengths = list(lengths)
    best_max = max(lengths)

    for _ in range(max_iterations):
        if is_balanced(lengths):
            break

        improved = False
        for boundary_idx in range(1, len(angles) - 1):
            for direction in (-1, 1):
                trial = list(angles)
                left_span = trial[boundary_idx] - trial[boundary_idx - 1]
                right_span = trial[boundary_idx + 1] - trial[boundary_idx]
                delta = min(left_span, right_span) * 0.05 * direction
                trial[boundary_idx] += delta
                if not (trial[boundary_idx - 1] < trial[boundary_idx] < trial[boundary_idx + 1]):
                    continue
                trial_lengths = length_fn(trial)
                if max(trial_lengths) < max(lengths):
                    angles = trial
                    lengths = trial_lengths
                    improved = True

        if max(lengths) < best_max:
            best_max = max(lengths)
            best_angles = list(angles)
            best_lengths = list(lengths)

        if not improved:
            break

    return best_angles, best_lengths
