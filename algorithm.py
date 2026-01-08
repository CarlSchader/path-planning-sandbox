from typing import Callable
import heapq

from node import Node
from metrics import euclidean

def predecessors(node: Node, obstacles: set[Node] = set()) -> list[Node]:
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


### A* Search Algorithm Implementation ###
class AStarPlanner:
    obstacles: set[Node]
    cost_metric: Callable[[Node, Node], float]
    heuristic_metric: Callable[[Node, Node], float]

    def __init__(
        self,
        start: Node,
        goal: Node,
        obstacles: set[Node] = set(),
        metric: Callable[[Node, Node], float] = euclidean,
    ):
        self.start = start
        self.goal = goal
        self.obstacles = obstacles
        self.metric = metric

    def _h(self, node: Node) -> float:
        return self.metric(node, self.goal)

    def _g(self, node: Node) -> float:
        return self.metric(self.start, node)

    def _f(self, node: Node) -> float:
        return self._g(node) + self._h(node)

    def execute(self) -> None | tuple[dict[Node, Node], set[Node]]:
        frontier: list[tuple[float, Node]] = []
        explored: set[Node] = set()
        heapq.heappush(frontier, (0.0, self.start))
        explored.add(self.start)

        came_from: dict[Node, Node] = {}

        while len(frontier) > 0:
            cost, current = heapq.heappop(frontier)
            for neighbor in predecessors(current, self.obstacles):
                if neighbor == self.goal:
                    came_from[neighbor] = current
                    return came_from, explored
                if neighbor not in explored:
                    explored.add(neighbor)
                    heapq.heappush(frontier, (self._f(neighbor), neighbor)) 
                    came_from[neighbor] = current
        return None

### LPA* Search Algorithm Implementation ###
def lpa_rhs(
    node: Node,
    g: Callable[[Node, Node], float],
    c: Callable[[Node, Node], float],
) -> float:
    if node == global_start:
        return 0.0
    else:
        return min(g(node, neighbor) + c(node, neighbor) for neighbor in get_neighbors(node))

def lpa_calculate_key(
    global_start: Node,
    node: Node,
    goal: Node,
    g: Callable[[Node, Node], float],
    c: Callable[[Node, Node], float],
) -> tuple[float, float]:
    return min(g(global_start, node), lpa_rhs(global_start, node, g, c)) + c(global_start, node), min(g(global_start, node), lpa_rhs(global_start, node, g, c))

def lpa_star_search(
    start: Node,
    goal: Node,
    obstacles: set[Node],
    cost_metric: Callable[[Node, Node], float] = euclidean,
    heuristic_metric: Callable[[Node, Node], float] = euclidean,
) -> None | tuple[dict[Node, Node], set[Node]]:
    frontier: list[tuple[tuple[float, float], Node]] = []
