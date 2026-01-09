from typing import Callable
import heapq

from node import Node
from metrics import euclidean

def successors(node: Node, obstacles: set[Node] = set()) -> list[Node]:
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
    start: Node
    goal: Node
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
            for neighbor in successors(current, self.obstacles):
                if neighbor == self.goal:
                    came_from[neighbor] = current
                    return came_from, explored
                if neighbor not in explored:
                    explored.add(neighbor)
                    heapq.heappush(frontier, (self._f(neighbor), neighbor)) 
                    came_from[neighbor] = current
        return None

### D* Search Algorithm Implementation ###
class DStarKey:
    def __init__(self, k1: float, k2: float):
        self.k1 = k1
        self.k2 = k2

    def __lt__(self, other: "DStarKey") -> bool:
        if self.k1 == other.k1:
            return self.k2 < other.k2
        return self.k1 < other.k1

    def __le__(self, other: "DStarKey") -> bool:
        return self.k1 < other.k1 or (self.k1 == other.k1 and self.k2 <= other.k2)

class DstarNode:
    def __init__(self, key, vertex):
        self.key = key
        self.vertex = vertex

    def __le__(self, other):
        return self.key <= other.key

    def __lt__(self, other):
        return self.key < other.key

class DStarQueue:
    def __init__(self):
        self.heap = []
        self.vertices_in_heap = []

    def top(self):
        return self.heap[0].vertex

    def top_key(self):
        if len(self.heap) == 0: return DStarKey(float('inf'), float('inf'))
        return self.heap[0].key

    def pop(self):
        """!!!THIS CODE WAS COPIED AND MODIFIED!!! Source: Lib/heapq.py"""
        """Pop the smallest item off the heap, maintaining the heap invariant."""
        lastelt = self.heap.pop()  # raises appropriate IndexError if heap is empty
        self.vertices_in_heap.remove(lastelt)
        if self.heap:
            returnitem = self.heap[0]
            self.heap[0] = lastelt
            self._siftup(0)
        else:
            returnitem = lastelt
        return returnitem

    def insert(self, vertex, key):
        item = DstarNode(key, vertex)
        self.vertices_in_heap.append(vertex)
        """!!!THIS CODE WAS COPIED AND MODIFIED!!! Source: Lib/heapq.py"""
        """Push item onto heap, maintaining the heap invariant."""
        self.heap.append(item)
        self._siftdown(0, len(self.heap) - 1)

    def contains(self, vertex):
        return vertex in self.vertices_in_heap

    def remove(self, vertex):
        self.vertices_in_heap.remove(vertex)
        for index, dstar_node in enumerate(self.heap):
            if dstar_node.vertex == vertex:
                self.heap[index] = self.heap[len(self.heap) - 1]
                self.heap.remove(self.heap[len(self.heap) - 1])
                break
        self.build_heap()

    def update(self, vertex, key):
        for index, dstar_node in enumerate(self.heap):
            if dstar_node.vertex == vertex:
                self.heap[index].key = key
                break
        self.build_heap()

    # !!!THIS FUNCTION WAS COPIED AND MODIFIED!!! Source: Lib/heapq.py
    def build_heap(self):
        """Transform list into a heap, in-place, in O(len(x)) time."""
        n = len(self.heap)
        # Transform bottom-up.  The largest index there's any point to looking at
        # is the largest with a child index in-range, so must have 2*i + 1 < n,
        # or i < (n-1)/2.  If n is even = 2*j, this is (2*j-1)/2 = j-1/2 so
        # j-1 is the largest, which is n//2 - 1.  If n is odd = 2*j+1, this is
        # (2*j+1-1)/2 = j so j-1 is the largest, and that's again n//2-1.
        for i in reversed(range(n // 2)):
            self._siftup(i)

    # !!!THIS FUNCTION WAS COPIED AND MODIFIED!!! Source: Lib/heapq.py
    # 'heap' is a heap at all indices >= startpos, except possibly for pos.  pos
    # is the index of a leaf with a possibly out-of-order value.  Restore the
    # heap invariant.
    def _siftdown(self, startpos, pos):
        newitem = self.heap[pos]
        # Follow the path to the root, moving parents down until finding a place
        # newitem fits.
        while pos > startpos:
            parentpos = (pos - 1) >> 1
            parent = self.heap[parentpos]
            if newitem < parent:
                self.heap[pos] = parent
                pos = parentpos
                continue
            break
        self.heap[pos] = newitem

    def _siftup(self, pos):
        endpos = len(self.heap)
        startpos = pos
        newitem = self.heap[pos]
        # Bubble up the smaller child until hitting a leaf.
        childpos = 2 * pos + 1  # leftmost child position
        while childpos < endpos:
            # Set childpos to index of smaller child.
            rightpos = childpos + 1
            if rightpos < endpos and not self.heap[childpos] < self.heap[rightpos]:
                childpos = rightpos
            # Move the smaller child up.
            self.heap[pos] = self.heap[childpos]
            pos = childpos
            childpos = 2 * pos + 1
        # The leaf at pos is empty now.  Put newitem there, and bubble it up
        # to its final resting place (by sifting its parents down).
        self.heap[pos] = newitem
        self._siftdown(startpos, pos)

class DStarPlanner:
    start: Node
    goal: Node
    obstacles: set[Node]
    metric: Callable[[Node, Node], float]
    g_map: dict[Node, float]
    rhs_map: dict[Node, float]
    U: DStarQueue
    k_m: float

    def __init__(
        self,
        start: Node,
        goal: Node,
        heuristic: Callable[[Node, Node], float] = euclidean,
        obstacles: set[Node] = set(),
    ):
        self.start: Node = start
        self.s_last: Node = start
        self.goal: Node = goal
        self.obstacles: set[Node] = obstacles
        self.g_map: dict[Node, float] = {}
        self.rhs_map: dict[Node, float] = {}
        self.U: DStarQueue = DStarQueue()
        self.h = heuristic
        self.k_m: float = 0.0

        self.rhs_map[self.goal] = 0.0
        self.U.insert(self.goal, DStarKey(heuristic(self.start, self.goal), 0.0))

    def g(self, node: Node) -> float:
        return self.g_map.get(node, float('inf'))

    def rhs(self, node: Node) -> float:
        return self.rhs_map.get(node, float('inf'))

    def calculate_key(self, node: Node) -> DStarKey:
        return DStarKey(
            min(self.g(node), self.rhs(node)) + self.h(self.start, node) + self.k_m, 
            min(self.g(node), self.rhs(node))
        )

    def update_vertex(self, u: Node) -> None:
        if self.g(u) != self.rhs(u) and self.U.contains(u):
            self.U.update(u, self.calculate_key(u))
        elif self.g(u) != self.rhs(u) and not self.U.contains(u):
            self.U.insert(u, self.calculate_key(u))
        elif self.g(u) == self.rhs(u) and self.U.contains(u):
            self.U.remove(u)

    def get_path(self, cost: Callable[[Node, Node], float]) -> list[Node]:
        path = []
        current = self.start
        while current != self.goal:
            path.append(current)
            next_node = min(
                successors(current, self.obstacles),
                key=lambda n: cost(current, n) + self.g(n),
            )
            if next_node is None or self.g(next_node) == float('inf'):
                return []  # No path found
            current = next_node
        path.append(self.goal)
        return path

    def execute(self, cost: Callable[[Node, Node], float]) -> set[Node]:
        explored: set[Node] = set() # extra
        while self.U.top_key() < self.calculate_key(self.start) or self.rhs(self.start) != self.g(self.start):
            u = self.U.top()
            explored.add(u) # extra
            k_old = self.U.top_key()
            k_new = self.calculate_key(u)
            if k_old < k_new:
                self.U.update(u, k_new)
            elif self.g(u) > self.rhs(u):
                self.g_map[u] = self.rhs(u)
                self.U.remove(u)
                for s in successors(u, self.obstacles):
                    if s != self.goal:
                        self.rhs_map[s] = min(self.rhs(s), cost(s, u) + self.g(u))
                        self.update_vertex(s)
            else:
                g_old = self.g(u)
                self.g_map[u] = float('inf')
                for s in successors(u, self.obstacles) + [u]:
                    if self.rhs(s) == cost(s, u) + g_old:
                        if s != self.goal:
                            self.rhs_map[s] = min([cost(s, s_prime) + self.g(s_prime) for s_prime in successors(s, self.obstacles)])
                    self.update_vertex(s)
        return explored # extra

    def update_on_new_cost(
        self,
        old_cost: Callable[[Node, Node], float],
        new_cost: Callable[[Node, Node], float],
        changed_edges: list[tuple[Node, Node]],
    ) -> None:
        self.k_m += self.h(self.s_last, self.start)
        self.s_last = self.start
        for (u, v) in changed_edges:
            c_old = old_cost(u, v)
            c_new = new_cost(u, v) 
            if c_old > c_new:
                if u != self.goal:
                    self.rhs_map[u] = min(self.rhs(u), c_new + self.g(v))
            elif self.rhs(u) == c_old + self.g(v):
                if u != self.goal:
                    self.rhs_map[u] = min([new_cost(u, s) + self.g(s) for s in successors(u, self.obstacles)])
            self.update_vertex(u)
