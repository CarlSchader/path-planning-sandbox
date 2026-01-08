from node import Node

def manhatten(node1: Node, node2: Node) -> float:
    return abs(node1.x - node2.x) + abs(node1.y - node2.y)
