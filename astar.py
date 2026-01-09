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
    metric: Callable[[Node, Node], float] = euclidean,
) -> None:
    while start != goal:
        planner = AStarPlanner(start, goal, obstacles, metric)
        result = planner.execute()
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
    parser.add_argument("-m", "--metric", type=str, choices=[m.value for m in Metric], default=Metric.EUCLIDEAN.value, help="Cost metric to use")

    args = parser.parse_args()

    if args.obstacles_file:
        obstacles = load_obstacles(args.obstacles_file)
    else:
        obstacles = set()

    start = Node()
    goal = Node(args.end_x, args.end_y)
    metric = Metric(args.metric).to_function()
    traverse(start, goal, obstacles, metric)

if __name__ == "__main__":
    main()
