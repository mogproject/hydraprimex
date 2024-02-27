import networkx as nx
from typing import TextIO, Optional

__all__ = [
    'read_pace_extended',
    'load_pace_extended',
    'write_pace_extended',
    'save_pace_extended',
]


def read_pace_extended(input: TextIO) -> tuple[nx.Graph, Optional[int], Optional[int]]:
    G = nx.Graph()
    for line in input.readlines():
        line = line.strip()
        if not line:
            continue
        if line.startswith('c'):
            continue  # ignore comments

        tokens = line.split()

        if tokens[0] == 'p':
            assert tokens[1] == 'twwe', f'unexpected problem descriptor: {tokens[1]}'
            assert 4 <= len(tokens) <= 6

            given_n = int(tokens[2])
            given_m = int(tokens[3])
            given_ub = int(tokens[4]) if len(tokens) >= 5 else None
            given_lb = int(tokens[5]) if len(tokens) >= 6 else None

        elif tokens[0] == 'v':
            assert len(tokens) == 2

            v = int(tokens[1])
            G.add_node(v)

        elif tokens[0] == 'e':
            assert 3 <= len(tokens) <= 4

            u = int(tokens[1])
            v = int(tokens[2])
            c = int(tokens[3]) if len(tokens) >= 4 else 0  # defaults to black edge (0)
            G.add_edge(u, v, color=c)

        else:
            assert False, f'unexpected line: {line}'

    assert given_n == G.number_of_nodes(), 'inconsistent n'
    assert given_m == G.number_of_edges(), 'inconsistent m'
    return G, given_ub, given_lb


def load_pace_extended(path: str) -> tuple[nx.Graph, Optional[int], Optional[int]]:
    with open(path) as f:
        return read_pace_extended(f)


def write_pace_extended(output: TextIO, G: nx.Graph, comments: list[str] = [], ub: Optional[int] = None, lb: Optional[int] = None, offset: int=0) -> None:
    problem = 'twwe'  # problem descriptor (TWin-Width Extended)
    n = G.number_of_nodes()
    m = G.number_of_edges()

    if ub is None and lb is not None:
        ub = n

    # comments
    for comment in comments:
        output.write(f'c {comment}\n')

    # p-line
    tokens = ['p', problem, str(n), str(m)] + ([] if ub is None else [str(ub)]) + ([] if lb is None else [str(lb)])
    output.write(' '.join(tokens) + '\n')

    # vertices (not necessarily consecutive)
    for v in G.nodes():
        output.write(f'v {v + offset}\n')

    # colored edges
    for u, v in G.edges():
        color = G.edges[u, v]['color']  # 0 or 1
        output.write(f'e {u + offset} {v + offset} {color}\n')


def save_pace_extended(path: str, G: nx.Graph, comments: list[str] = []) -> None:
    with open(path, 'w') as f:
        write_pace_extended(f, G, comments)
