import networkx as nx


def degeneracy(G: nx.Graph) -> tuple[int, list[int]]:
    """
    Returns the degenecary and a degeneracy ordering of the given graph.

    @param G input graph
    @return pair of degeneracy and list of vertices in degeneracy ordering

    Time complexity: O(n+m)
    """
    n = len(G)
    assert all(0 <= i < n for i in G), 'vertices must be labeled from 0 to n-1'

    # initialization
    degen = 0
    deg = [0 for _ in range(n)]  # stores the current degree of each vertex: Vertex -> Degree
    deg_idx = [0 for _ in range(n)]  # index of the first degree-i vertex: Vertex -> Index
    ret: list[int] = []  # stores degeneracy ordering: Index -> Vertex
    ret_inv = [0 for _ in range(n)]  # mapping of vertex labels to indices in `ret`; i.e. the preimage of `ret`: Vertex -> Index

    bucket: list[list[int]] = [[] for _ in range(n)]  # temporary data structure
    for v, d in G.degree():
        bucket[d] += [v]
        deg[v] = d

    for d, vs in enumerate(bucket):
        deg_idx[d] = len(ret)
        for v in vs:
            ret_inv[v] = len(ret)
            ret += [v]

    # iteratively remove a vertex with the minimum degree
    for i in range(n):
        # update degeneracy
        v = ret[i]
        degen = max(degen, deg[v])

        # remove vertex v from the graph
        for u in G[v]:
            j = ret_inv[u]
            if j < i:  # already removed from the graph
                continue

            assert i < j

            # swap the position of u with the first vertex in the same bucket
            swap_with = max(i + 1, deg_idx[deg[u]])
            if j != swap_with:
                ret[swap_with], ret[j] = u, ret[swap_with]
                ret_inv[ret[swap_with]], ret_inv[ret[j]] = swap_with, j

            # shift the starting index
            deg_idx[deg[u]] += 1

            # decrease degree by one
            deg[u] -= 1

    return degen, ret
