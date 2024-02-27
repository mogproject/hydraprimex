"""
Recursive definition of the twin-width.
"""
import networkx as nx
from functools import cache
from algorithms.common.contraction import *


def compute_twinwidth(G: nx.Graph, contraction_seqence: list[tuple[int, int]]) -> int:
    n = len(G)
    if n == 0:
        return 0
    assert len(contraction_seqence) == n - 1

    # normalize the given contraction sequence
    seq = normalize_contraction_sequence(contraction_seqence)
    phi = [v for _, v in seq] + [n - 1]  # order [0,n) -> vertex[0,n)

    @cache
    def r(t: int, i: int, j: int) -> int:
        """
        Returns 1 if there is a red edge between vertices `i` and `j` at time `t`
        i.e. after contracting `t` vertex pairs; 0 otherwise.
        """
        assert t <= i < j < n, f't={t}, i={i}, j={j}'

        if t == 0:
            return 0

        # already red
        if r(t - 1, i, j) == 1:
            return 1

        p, v = seq[t - 1]  # previous contraction
        assert p > v  # v is merged into p
        assert phi[t - 1] == v

        for ii, jj in [(i, j), (j, i)]:
            if phi[ii] == p:
                # transfer
                if r(t - 1, t - 1, jj) == 1:
                    return 1

                # symmetric difference
                q = phi[jj]
                if G.has_edge(q, v) != G.has_edge(q, p):
                    return 1
        return 0

    ret = 0
    for t in range(1, n):
        for i in range(t, n):
            ret = max(ret, sum(r(t, min(i, j), max(i, j)) for j in range(t, n) if i != j))
    return ret
