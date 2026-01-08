import heapq
from typing import Callable
import time

from node import Node
from metrics import *
from view import render_grid

def get_neighbors(node: Node, obstacles: set[Node] = set()) -> list[Node]:
    candidates = [
        Node(node.x + 1, node.y),
        Node(node.x + 1, node.y + 1),
        Node(node.x, node.y + 1),
        Node(node.x - 1, node.y + 1),
        Node(node.x - 1, node.y),
        Node(node.x - 1, node.y - 1),
        Node(node.x, node.y - 1),
        Node(node.x + 1, node.y - 1),
    ]
    return [c for c in candidates if c not in obstacles]

def heuristic(start: Node, end: Node, metric: Callable[[Node, Node], float]) -> float:
    return metric(start, end)

def cost_function(start: Node, end: Node, metric: Callable[[Node, Node], float]) -> float:
    return metric(start, end)

def total_cost(
    start: Node,
    goal: Node,
    current: Node,
    cost_metric: Callable[[Node, Node], float] = manhatten,
    heuristic_metric: Callable[[Node, Node], float] = manhatten,
) -> float:
    return cost_function(start, current, cost_metric) + heuristic(current, goal, heuristic_metric)

def a_star_search(
    start: Node,
    goal: Node,
    obstacles: set[Node],
    cost_metric: Callable[[Node, Node], float] = manhatten,
    heuristic_metric: Callable[[Node, Node], float] = manhatten,
) -> None | tuple[dict[Node, Node], set[Node]]:
    frontier: list[tuple[float, Node]] = []
    explored: set[Node] = set()
    heapq.heappush(frontier, (0.0, start))
    explored.add(start)

    came_from: dict[Node, Node] = {}

    while len(frontier) > 0:
        cost, current = heapq.heappop(frontier)
        for neighbor in get_neighbors(current, obstacles):
            if neighbor == goal:
                came_from[neighbor] = current
                return came_from, explored
            if neighbor not in explored:
                explored.add(neighbor)
                heapq.heappush(frontier, (total_cost(start, goal, neighbor, cost_metric, heuristic_metric), neighbor)) 
                came_from[neighbor] = current
    return None

def load_obstacles(file_path: str) -> set[Node]:
    obstacles: set[Node] = set()
    with open(file_path, 'r') as f:
        for line in f:
            x_str, y_str = line.strip().split(' ')
            obstacles.add(Node(int(x_str), int(y_str)))
    return obstacles

def reconstruct_path(came_from: dict[Node, Node], start: Node, goal: Node) -> list[Node]:
    current: Node = goal
    path: list[Node] = []
    while current != start:
        path.append(current)
        current = came_from[current]
    path.append(start)
    path.reverse()
    return path

def traverse(
    start: Node,
    goal: Node,obstacles: set[Node],
    cost_metric: Callable[[Node, Node], float] = manhatten,
    heuristic_metric: Callable[[Node, Node], float] = manhatten,
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
