from typing import Callable

from node import Node
import enum

def zero(node1: Node, node2: Node) -> float:
    return 0.0

def manhatten(node1: Node, node2: Node) -> float:
    return abs(node1.x - node2.x) + abs(node1.y - node2.y)

def euclidean(node1: Node, node2: Node) -> float:
    return ((node1.x - node2.x) ** 2 + (node1.y - node2.y) ** 2) ** 0.5

class Metric(enum.Enum):
    ZERO = "zero"
    MANHATTEN = "manhatten"
    EUCLIDEAN = "euclidean"

    def to_function(self) -> Callable[[Node, Node], float]:
        if self == Metric.ZERO:
            return zero
        elif self == Metric.MANHATTEN:
            return manhatten
        elif self == Metric.EUCLIDEAN:
            return euclidean
        else:
            raise ValueError(f"Unknown metric: {self}")
