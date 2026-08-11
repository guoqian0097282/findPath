from __future__ import annotations

import numpy as np
from scipy.optimize import linear_sum_assignment


def linear_assignment(
    cost: np.ndarray,
    thresh: float,
) -> tuple[list[tuple[int, int]], list[int], list[int]]:
    nr, nc = cost.shape
    if nr == 0 or nc == 0:
        return [], list(range(nr)), list(range(nc))

    r, c = linear_sum_assignment(cost)
    matches: list[tuple[int, int]] = []
    matched_r: set[int] = set()
    matched_c: set[int] = set()
    for i, j in zip(r.tolist(), c.tolist()):
        if float(cost[i, j]) <= float(thresh):
            matches.append((i, j))
            matched_r.add(i)
            matched_c.add(j)

    u_r = [i for i in range(nr) if i not in matched_r]
    u_c = [j for j in range(nc) if j not in matched_c]
    return matches, u_r, u_c

