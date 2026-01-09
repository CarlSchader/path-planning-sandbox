import heapq
from typing import Callable
import time

from node import Node
from metrics import euclidean, Metric
from view import render_grid
from helpers import reconstruct_path, load_obstacles
from algorithm import *

COST_UPDATE_INTERVAL = 10

def update_cost(
    mertric: Callable[[Node, Node], float],
    obstacles: set[Node],
    iteration: int
) -> tuple[Callable[[Node, Node], float], list[tuple[Node, Node]]]:
    if iteration % COST_UPDATE_INTERVAL == 0:
        def cost(n1: Node, n2: Node) -> float:
            if n2 in obstacles:
                return float('inf')
            base_cost = mertric(n1, n2)
            if n1.y == n2.y:
                return base_cost * 1000
            return base_cost
    else:
        def cost(n1: Node, n2: Node) -> float:
            if n2 in obstacles:
                return float('inf')
            return mertric(n1, n2)

    changed_edges = []
    for x in range(-20, 21):
        for y in range(-20, 21):
            n1 = Node(x, y)
            for n2 in successors(n1, obstacles):
                changed_edges.append((n1, n2))

    return cost, changed_edges

def traverse(
    start: Node,
    goal: Node,
    obstacles: set[Node],
    metric: Callable[[Node, Node], float] = euclidean,
) -> None:
    s_last = start
    heuristic = metric
    iteration = 1
    cost, _ = update_cost(metric, obstacles, iteration)
    planner = DStarPlanner(start, goal, heuristic, obstacles)
    explored = planner.execute(cost)
    while start != goal:
        if planner.g(start) == float('inf'):
            print("No path found!")
            return

        # start = min(successors(start, obstacles), key=lambda n: cost(start, n) + planner.g(n))
        path = planner.get_path(cost)

        render_grid(start, goal, path, explored, obstacles)
        time.sleep(0.25)

        start = path[1]

        # scan for changes in edge cost
        if iteration % COST_UPDATE_INTERVAL == 0:
            old_cost = cost
            cost, changed_edges = update_cost(metric, obstacles, iteration)
            print(f"Updating costs at iteration {iteration}")

        planner.start = start
        explored = planner.execute(cost)
        iteration += 1

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
