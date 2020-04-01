from __future__ import print_function
from heapq import * #Hint: Use heappop and heappush

ACTIONS = [(0,-1),(-1,0),(0,1),(1,0)]

class Agent:
    def __init__(self, grid, start, goal, type):
        self.grid = grid
        self.start = start 
        self.grid.nodes[start].start = True
        self.goal = goal
        self.grid.nodes[goal].goal = True
        self.final_cost = 0
        self.search(type)
    def search(self, type):
        self.finished = False
        self.failed = False
        self.type = type
        self.previous = {}
        if self.type == "dfs":
            self.frontier = [self.start]
            self.explored = []
        elif self.type == "bfs":
            pass
        elif self.type == "ucs":
            pass
        elif self.type == "astar":
            pass
    def show_result(self):
        current = self.goal
        while not current == self.start:
            current = self.previous[current]
            self.grid.nodes[current].in_path = True #This turns the color of the node to red
    def make_step(self):
        if self.type == "dfs":
            self.dfs_step()
        elif self.type == "bfs":
            self.bfs_step()
        elif self.type == "ucs":
            self.ucs_step()
        elif self.type == "astar":
            self.astar_step()

    # Explores the next node according to the DFS algorithm.
    def dfs_step(self):
        # Checks if the frontier is empty, which indicates we have exhausted all nodes/paths.
        if not self.frontier:
            self.failed = True
            print("no path")
            return
        # Remove the next node to explore from the frontier (last node).
        current = self.frontier.pop()
        self.grid.nodes[current].checked = True
        self.grid.nodes[current].frontier = False
        self.explored.append(current)
        # Iterate through the neighboring nodes to check what new paths can be taken.
        children = [(current[0]+a[0], current[1]+a[1]) for a in ACTIONS]
        for node in children:
            # See what happens if you disable this check here
            if node in self.explored or node in self.frontier:
                continue
            # Checks whether the node is within bounds of the graph.
            if node[0] in range(self.grid.row_range) and node[1] in range(self.grid.col_range):
                # Checks whether the node is a puddle (which cannot be included in the path).
                if not self.grid.nodes[node].puddle:
                    # Assigns the path to this node (through this current node).
                    self.previous[node] = current
                    # Finishes search if we've found the goal.
                    if node == self.goal:
                        self.finished = True
                    # Given we haven't found the goal yet, add this node to the frontier to
                    # explore later.
                    else:
                        self.frontier.append(node)
                        # Marks the node to be a frontier node.
                        self.grid.nodes[node].frontier = True
    # Explores the next node according to the BFS algorithm.
    def bfs_step(self):
        self.failed = True
    # Explores the next node according to the UCS algorithm.
    def ucs_step(self):
        self.failed = True
    # Explores the next node according to the A* algorithm.
    def astar_step(self):
        self.failed = True
