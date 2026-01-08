from collections import deque

class Node:
    x: int
    y: int
    
    def __init__(self, x: int = 0, y: int = 0):
        self.x = x
        self.y = y

    def __eq__(self, other):
        return self.x == other.x and self.y == other.y

    def __hash__(self):
        return hash((self.x, self.y))

    def __repr__(self):
        return f"Node({self.x}, {self.y})"

def get_neighbors(node: Node, obstacles: set[Node] = set()) -> list[Node]:
    candidates = [Node(node.x + 1, node.y), Node(node.x - 1, node.y),
            Node(node.x, node.y + 1), Node(node.x, node.y - 1)]
    return [c for c in candidates if c not in obstacles]

def heuristic(node: Node) -> float:
    return 0

def cost_function(node: Node, alpha: float) -> float:
    return 1

def draw_grid(start: Node, goal: Node, path: list[Node], explored: set[Node], obstacles: set[Node], padding: int = 5) -> str:
    # Find the boundaries of the grid based on start, goal, and path
    all_points = [start, goal] + path
    min_x = min(node.x for node in all_points)
    max_x = max(node.x for node in all_points)
    min_y = min(node.y for node in all_points)
    max_y = max(node.y for node in all_points)
    
    # Add padding
    min_x -= padding
    max_x += padding
    min_y -= padding
    max_y += padding
    
    # Draw the grid
    grid_str = ""
    for y in range(max_y, min_y - 1, -1):
        row = ""
        for x in range(min_x, max_x + 1):
            node = Node(x, y)
            if node == start:
                row += "S "
            elif node == goal:
                row += "G "
            elif node in path:
                row += "P "
            elif node in obstacles:
                row += "# "
            elif node in explored:
                row += "_ "
            else:
                row += "  "
        grid_str += row + "\n"
    return grid_str

def a_star_search(start: Node, goal: Node, obstacles: set[Node], alpha: float) -> None | tuple[dict[Node, Node], set[Node]]:
    frontier: deque = deque()
    explored: set[Node] = set()
    frontier.append(start)
    explored.add(start)

    came_from: dict[Node, Node] = {}

    while len(frontier) > 0:
        current = frontier.popleft()
        for neighbor in get_neighbors(current, obstacles):
            if neighbor == goal:
                came_from[neighbor] = current
                return came_from, explored
            if neighbor not in explored:
                explored.add(neighbor)
                frontier.append(neighbor)
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

def main():
    import argparse
    parser = argparse.ArgumentParser(description="Process some parameters.")
    parser.add_argument("end_x", type=int, help="X coordinate of the end node")
    parser.add_argument("end_y", type=int, help="Y coordinate of the end node")
    parser.add_argument("-a", "--alpha", type=float, default=1.0, help="Alpha parameter for cost function")
    parser.add_argument("-o", "--obstacles_file", type=str, default=None, help="Path to obstacles file")

    args = parser.parse_args()

    if args.obstacles_file:
        obstacles = load_obstacles(args.obstacles_file)
    else:
        obstacles = set()

    start = Node()
    goal = Node(args.end_x, args.end_y)
    result = a_star_search(start, goal, obstacles, args.alpha)

    if result is not None:
        came_from, explored = result
        path = reconstruct_path(came_from, start, goal)
        print(draw_grid(start, goal, path, explored, obstacles))
    else:
        print("No path found")

if __name__ == "__main__":
    main()
