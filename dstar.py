import heapq
from typing import Callable
import time
import cProfile
import pstats
import io

from node import Node
from metrics import euclidean, Metric
from view import render_grid
from helpers import reconstruct_path, load_obstacles
from algorithm import *

# Sensor range - robot discovers obstacles within this distance
SENSOR_RANGE = 5

# How often to add new obstacles (every N steps)
OBSTACLE_INTERVAL = 15


def make_cost(
    metric: Callable[[Node, Node], float], obstacles: set[Node]
) -> Callable[[Node, Node], float]:
    # Copy obstacles to capture current state (avoid reference issues)
    obstacles_snapshot = frozenset(obstacles)

    def cost(n1: Node, n2: Node) -> float:
        if n2 in obstacles_snapshot:
            return float("inf")
        return metric(n1, n2)

    return cost


def generate_dynamic_obstacles(step: int, goal: Node) -> list[Node]:
    """Generate obstacles that appear at certain steps to block the direct path."""
    new_obstacles = []

    # Add walls that appear at different times, blocking the diagonal path
    # Walls have gaps to ensure path exists
    if step == OBSTACLE_INTERVAL:
        # Wall 1: vertical wall at x=20, gap at y=10
        for y in range(12, 25):
            if y != 18:  # Leave a gap
                new_obstacles.append(Node(20, y))

    if step == OBSTACLE_INTERVAL * 2:
        # Wall 2: horizontal wall at y=40, gap at x=42
        for x in range(35, 48):
            if x != 42:  # Leave a gap
                new_obstacles.append(Node(x, 40))

    if step == OBSTACLE_INTERVAL * 3:
        # Wall 3: vertical wall at x=60, gap at y=58
        for y in range(52, 65):
            if y != 58:  # Leave a gap
                new_obstacles.append(Node(60, y))

    if step == OBSTACLE_INTERVAL * 4:
        # Wall 4: horizontal wall at y=80, gap at x=82
        for x in range(75, 88):
            if x != 82:  # Leave a gap
                new_obstacles.append(Node(x, 80))

    return new_obstacles


def traverse(
    start: Node,
    goal: Node,
    obstacles: set[Node],
    metric: Callable[[Node, Node], float] = euclidean,
    timing: bool = False,
) -> None:
    start_time = time.time()
    obstacles = obstacles.copy()  # Don't modify original

    heuristic = metric
    cost = make_cost(metric, obstacles)
    planner = DStarPlanner(start, goal, heuristic, obstacles)
    explored = planner.execute(cost)
    path = planner.get_path(cost)

    step = 0
    replan_count = 0

    while start != goal:
        if planner.g(start) == float("inf"):
            print("No path found!")
            return

        if not timing:
            render_grid(start, goal, path, explored, obstacles)
            time.sleep(0.1)

        # Move to next position
        start = path[1]
        path = path[1:]
        planner.start = start
        step += 1

        # Simulate discovering new obstacles
        new_obstacles = generate_dynamic_obstacles(step, goal)

        if new_obstacles:
            # Add new obstacles
            old_cost = cost
            for obs in new_obstacles:
                obstacles.add(obs)
                planner.obstacles.add(obs)
            cost = make_cost(metric, obstacles)

            # Compute changed edges (edges to/from new obstacles)
            changed_edges = []
            for obs in new_obstacles:
                for neighbor in successors(
                    obs, set()
                ):  # Get all neighbors, ignore obstacles for this
                    changed_edges.append((neighbor, obs))
                    changed_edges.append((obs, neighbor))

            # Incremental update
            planner.update_on_new_cost(old_cost, cost, changed_edges)
            explored = planner.execute(cost)
            path = planner.get_path(cost)
            replan_count += 1

    elapsed_time = time.time() - start_time
    if timing:
        print(f"Total time taken: {elapsed_time:.4f} seconds")
        print(f"Replans: {replan_count}")


def main():
    import argparse

    parser = argparse.ArgumentParser(description="Process some parameters.")
    parser.add_argument("end_x", type=int, help="X coordinate of the end node")
    parser.add_argument("end_y", type=int, help="Y coordinate of the end node")
    parser.add_argument(
        "-o", "--obstacles_file", type=str, default=None, help="Path to obstacles file"
    )
    parser.add_argument(
        "-m",
        "--metric",
        type=str,
        choices=[m.value for m in Metric],
        default=Metric.EUCLIDEAN.value,
        help="Cost metric to use",
    )
    parser.add_argument(
        "-t", "--timing", action="store_true", help="Enable timing output"
    )

    args = parser.parse_args()

    if args.obstacles_file:
        obstacles = load_obstacles(args.obstacles_file)
    else:
        obstacles = set()

    start = Node()
    goal = Node(args.end_x, args.end_y)
    metric = Metric(args.metric).to_function()

    if args.timing:
        profiler = cProfile.Profile()
        profiler.enable()
        traverse(start, goal, obstacles, metric, timing=args.timing)
        profiler.disable()

        # Print profiling results sorted by cumulative time
        stream = io.StringIO()
        stats = pstats.Stats(profiler, stream=stream).sort_stats("cumulative")
        stats.print_stats(30)
        print(stream.getvalue())
    else:
        traverse(start, goal, obstacles, metric, timing=args.timing)


if __name__ == "__main__":
    main()
