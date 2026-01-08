import heapq
from typing import Callable
import time

from node import Node
from metrics import euclidean, Metric
from view import render_grid
from helpers import reconstruct_path, load_obstacles
from algorithm import *

def traverse(
    start: Node,
    goal: Node,
    obstacles: set[Node],
    cost_metric: Callable[[Node, Node], float] = euclidean,
    heuristic_metric: Callable[[Node, Node], float] = euclidean,
) -> None:
    while start != goal:
        result = a_star_search(start, goal, obstacles, cost_metric, heuristic_metric)
        if result is None:
            return
        came_from, explored = result
        path = reconstruct_path(came_from, start, goal)

        render_grid(start, goal, path, explored, obstacles)
        time.sleep(0.25)

        start = path[1]
    return

def main():
    import argparse
    parser = argparse.ArgumentParser(description="Process some parameters.")
    parser.add_argument("end_x", type=int, help="X coordinate of the end node")
    parser.add_argument("end_y", type=int, help="Y coordinate of the end node")
    parser.add_argument("-o", "--obstacles_file", type=str, default=None, help="Path to obstacles file")
    parser.add_argument("-C", "--cost_metric", type=str, choices=[m.value for m in Metric], default=Metric.EUCLIDEAN.value, help="Cost metric to use")
    parser.add_argument("-H", "--heuristic_metric", type=str, choices=[m.value for m in Metric], default=Metric.EUCLIDEAN.value, help="Heuristic metric to use")

    args = parser.parse_args()

    if args.obstacles_file:
        obstacles = load_obstacles(args.obstacles_file)
    else:
        obstacles = set()

    start = Node()
    goal = Node(args.end_x, args.end_y)
    cost_metric = Metric(args.cost_metric).to_function()
    heuristic_metric = Metric(args.heuristic_metric).to_function()
    traverse(start, goal, obstacles, cost_metric, heuristic_metric)

if __name__ == "__main__":
    main()
