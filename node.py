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

    def __lt__(self, other):
        return (self.x, self.y) < (other.x, other.y)

