from node import Node
import os

def draw_grid(
    start: Node,
    goal: Node,
    path: list[Node],
    explored: set[Node],
    obstacles: set[Node],
    padding: int = 5
) -> str:
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

def render_grid(
    start: Node,
    goal: Node, 
    path: list[Node],
    explored: set[Node],
    obstacles: set[Node],
    padding: int = 5
) -> None:
    os.system('cls' if os.name == 'nt' else 'clear')
    grid_str = draw_grid(start, goal, path, explored, obstacles, padding)
    print(grid_str)
