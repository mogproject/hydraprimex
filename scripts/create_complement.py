#!/usr/bin/env python3
"""
Output the complement graph in the PACE 2023 format.

Usage:
  create_complement.py [options] GRAPH_PATH

Examples:
  ./scripts/create_complement.py path/to/graph.gr
"""

import sys
import os
import argparse
import networkx as nx

__version__ = '0.0.1'
__license__ = 'Apache License, Version 2.0'


# Path settings.
SCRIPT_PATH = os.path.realpath(__file__)
SCRIPT_DIR = os.path.dirname(SCRIPT_PATH)
PROJECT_DIR = os.path.dirname(SCRIPT_DIR)
PYTHON_MAIN = os.path.join(PROJECT_DIR, 'src', 'main', 'python')

if PYTHON_MAIN not in sys.path:
    sys.path.insert(0, PYTHON_MAIN)

from readwrite import load_pace_2023, write_pace_2023
from util.ColoredLogger import *


def get_parser():
    """Argument parser."""

    parser = argparse.ArgumentParser(description='Simulate contractions and output a partial trigraph.')
    parser.add_argument('-v', '--version', action='version', version=f'%(prog)s {__version__}')
    parser.add_argument('graph_path', help='input graph file')
    parser.add_argument('--log-level', choices=LOG_LEVELS.keys(), default='info', help='log level')
    parser.add_argument('--no-color', action='store_true', help='disables log coloring')
    return parser


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

    # save complement graph
    H = nx.complement(G)
    logger.info(f'Writing complement graph: n={H.number_of_nodes()}, m={H.number_of_edges()}')
    write_pace_2023(sys.stdout, H)


if __name__ == '__main__':
    main(get_parser().parse_args())
