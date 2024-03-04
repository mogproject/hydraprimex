#!/usr/bin/env python3
"""
Simulates the first few steps of contractions and outputs a trigraph.

Usage:
  create_trigraph.py [-s STEPS] [options] GRAPH_PATH CONTRACTION_PATH

Examples:
  ./scripts/create_trigraph.py -s NUM_STEPS path/to/graph.gr path/to/contraction.cs
"""

import sys
import os
import argparse

__version__ = '0.0.1'
__license__ = 'Apache License, Version 2.0'


# Path settings.
SCRIPT_PATH = os.path.realpath(__file__)
SCRIPT_DIR = os.path.dirname(SCRIPT_PATH)
PROJECT_DIR = os.path.dirname(SCRIPT_DIR)
PYTHON_MAIN = os.path.join(PROJECT_DIR, 'src', 'main', 'python')

if PYTHON_MAIN not in sys.path:
    sys.path.insert(0, PYTHON_MAIN)

from readwrite import load_pace_2023, write_pace_extended
from util.ColoredLogger import *
from algorithms.common.contraction import contract_with_history, initialize_trigraph


def get_parser():
    """Argument parser."""

    parser = argparse.ArgumentParser(description='Simulate contractions and output a partial trigraph.')
    parser.add_argument('-v', '--version', action='version', version=f'%(prog)s {__version__}')
    parser.add_argument('graph_path', help='input graph file')
    parser.add_argument('contraction_path', help='input contraction file')
    parser.add_argument('-s', '--steps', type=int, default=0, help='number of steps to simulate (default: 0)')
    parser.add_argument('--log-level', choices=LOG_LEVELS.keys(), default='info', help='log level')
    parser.add_argument('--no-color', action='store_true', help='disables log coloring')
    return parser


def load_contractions(path):
    contractions = []
    with open(path) as f:
        for line in f:
            if line.startswith('c'):
                continue
            u, v = map(int, line.strip().split())
            contractions += [(u - 1, v - 1)]
    return contractions


def main(args):
    """Entry point of the program."""

    # logger settings
    log_level = LOG_LEVELS[args.log_level]
    logger = get_logger(__name__, log_level, not args.no_color)

    # load input file
    G = load_pace_2023(args.graph_path)
    n = G.number_of_nodes()
    m = G.number_of_edges()
    logger.info(f'Loaded graph: path={args.graph_path}, n={n}, m={m}')

    # load contraction file
    cs = load_contractions(args.contraction_path)
    logger.info(f'Loaded contractions: path={args.contraction_path}')

    # simulate contractions
    steps = args.steps
    if steps > n - 1:
        logger.warning(f'Too large steps. Reset to: {n - 1}')
        steps = min(n - 1, args.steps)

    initialize_trigraph(G)
    max_red_deg = 0
    for i in range(steps):
        max_red_deg = max(max_red_deg, contract_with_history(G, cs[i][0], cs[i][1]))

    logger.info(f'Simulated contractions: steps={steps}, max_red_deg={max_red_deg}')

    # output trigraph
    red_deg = max(sum(G.edges[u, v]['color'] for u in G[v]) for v in G)
    logger.info(f'Writing trigraph: n={G.number_of_nodes()}, m={G.number_of_edges()}, red_deg={red_deg}')
    write_pace_extended(sys.stdout, G, lb=red_deg, offset=1)


if __name__ == '__main__':
    main(get_parser().parse_args())
