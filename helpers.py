from node import Node

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

def load_obstacles(file_path: str) -> set[Node]:
    obstacles: set[Node] = set()
    with open(file_path, 'r') as f:
        for line in f:
            x_str, y_str = line.strip().split(' ')
            obstacles.add(Node(int(x_str), int(y_str)))
    return obstacles

def shift_obstacles(obstacles: set[Node], dx: int, dy: int) -> set[Node]:
    return {Node(node.x + dx, node.y + dy) for node in obstacles}

def reconstruct_path(came_from: dict[Node, Node], start: Node, goal: Node) -> list[Node]:
    current: Node = goal
    path: list[Node] = []
    while current != start:
        path.append(current)
        current = came_from[current]
    path.append(start)
    path.reverse()
    return path

