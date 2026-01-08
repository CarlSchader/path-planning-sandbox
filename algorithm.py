from typing import Callable
import heapq

from node import Node
from metrics import euclidean
from helpers import get_neighbors

def heuristic(start: Node, end: Node, metric: Callable[[Node, Node], float]) -> float:
    return metric(start, end)

def cost_function(start: Node, end: Node, metric: Callable[[Node, Node], float]) -> float:
    return metric(start, end)

def total_cost(
    start: Node,
    goal: Node,
    current: Node,
    cost_metric: Callable[[Node, Node], float] = euclidean,
    heuristic_metric: Callable[[Node, Node], float] = euclidean,
) -> float:
    return cost_function(start, current, cost_metric) + heuristic(current, goal, heuristic_metric)

def a_star_search(
    start: Node,
    goal: Node,
    obstacles: set[Node],
    cost_metric: Callable[[Node, Node], float] = euclidean,
    heuristic_metric: Callable[[Node, Node], float] = euclidean,
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

def lpa_star_search(
    start: Node,
    goal: Node,
    obstacles: set[Node],
    cost_metric: Callable[[Node, Node], float] = euclidean,
    heuristic_metric: Callable[[Node, Node], float] = euclidean,
) -> None | tuple[dict[Node, Node], set[Node]]:
    pass  # Placeholder for LPA* implementation
