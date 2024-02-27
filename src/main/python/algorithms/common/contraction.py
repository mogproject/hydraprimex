"""
Contruction operation for trigraphs.
"""

import networkx as nx

__all__ = [
    'BLACK', 'RED',
    'initialize_trigraph',
    'contract_with_history',
    'rollback_history',
    'verify_contraction_sequence',
    'normalize_contraction_sequence',
    'is_red_edge',
]

# edge colors
BLACK = 0
RED = 1


def initialize_trigraph(G: nx.Graph, create_copy: bool = False) -> nx.Graph:
    if create_copy:
        G = G.copy()
    G.graph['history'] = []
    for e in G.edges():
        G.edges[e]['color'] = BLACK
    for v in G.nodes():
        G.nodes[v]['rdeg'] = 0
    return G


def get_black_neighbors(G: nx.Graph, v: int) -> list[int]:
    return [u for u in G[v] if G.edges[v, u]['color'] == BLACK]


def contract_with_history(G: nx.Graph, i, j) -> int:
    hist = {
        'i': i,
        'j': j,  # vertex to be merged
        # j's original neighbors with color information
        'removed': [(u, G.edges[j, u]['color']) for u in G[j]],
        # i's new neighbors connected with red edges
        'new': list(set(G.neighbors(j)) - set(G.neighbors(i)) - {i}),
        # i's original neighbors that used to be black
        'recolor': [v for v in set(G.neighbors(i)) - set(get_black_neighbors(G, j)) - {j} if G.edges[i, v]['color'] == BLACK]
    }
    G.graph['history'] += [hist]

    # make changes
    for u in G[j]:
        if G.edges[j, u]['color'] == RED:
            G.nodes[u]['rdeg'] -= 1
    G.remove_node(j)
    for v in hist['recolor']:
        G.edges[i, v]['color'] = RED
        G.nodes[i]['rdeg'] += 1
        G.nodes[v]['rdeg'] += 1
    for v in hist['new']:
        G.nodes[i]['rdeg'] += 1
        G.nodes[v]['rdeg'] += 1
    G.add_edges_from((i, v, dict(color=RED)) for v in hist['new'])

    # compute the red-degree
    closed_nbrs = list(G.neighbors(i)) + [i]
    return max(G.nodes[v]['rdeg'] for v in closed_nbrs)


def rollback_history(G: nx.Graph):
    # retrieve the latest event
    hist = G.graph['history'].pop()
    i = hist['i']
    j = hist['j']

    # make changes
    for v in hist['new']:
        G.nodes[i]['rdeg'] -= 1
        G.nodes[v]['rdeg'] -= 1
    G.remove_edges_from((i, v) for v in hist['new'])

    for v in hist['recolor']:
        G.nodes[i]['rdeg'] -= 1
        G.nodes[v]['rdeg'] -= 1
        G.edges[i, v]['color'] = BLACK
    G.add_node(j)
    G.nodes[j]['rdeg'] = 0
    for u, c in hist['removed']:
        if c == RED:
            G.nodes[j]['rdeg'] += 1
            G.nodes[u]['rdeg'] += 1
    G.add_edges_from((j, u, dict(color=c)) for u, c in hist['removed'])


def verify_contraction_sequence(G: nx.Graph, seq: list[tuple[int, int]]) -> int:
    """
    Verifies that a given sequence is a valid contraction sequence for the given graph
    and returns the maximum red degree for the sequence.

    @param G   input NetworkX Graph instance
    @param seq contraction sequence to test

    @return maximum red degree while performing the given contraction sequence
    @throw AssertionError when the given sequence is invalid
    """
    n = len(G)
    if n == 0:  # empty graph
        assert not seq
        return 0

    assert len(seq) == n - 1, f'incomplete sequence: len(seq) should be {n - 1} but is {len(seq)}.'

    # create a copy so that the given graph will not be modified
    G = initialize_trigraph(G, create_copy=True)
    red_deg = 0
    for t, (i, j) in enumerate(seq):
        assert G.has_node(i), f'node {i} does not exist in the graph at time {t}'
        assert G.has_node(j), f'node {j} does not exist in the graph at time {t}'
        red_deg = max(red_deg, contract_with_history(G, i, j))

    # OK
    return red_deg


def normalize_contraction_sequence(seq: list[tuple[int, int]]) -> list[tuple[int, int]]:
    """
    Make a valid elimination ordering, which satisfies u > v for every elimination pair (u, v).

    @param seq valid contraction sequence of vertices [0,n) (size should be n-1)
    @return normalized contraction sequence
    """
    ret = []
    vertices = list(range(len(seq) + 1))
    for i, j in seq:
        u, v = max(vertices[i], vertices[j]), min(vertices[i], vertices[j])
        vertices[i], vertices[j] = u, v
        ret += [(u, v)]
    return ret


def is_red_edge(G: nx.Graph, i: int, j: int) -> bool:
    return G.has_edge(i, j) and G.edges[i, j]['color'] == RED

