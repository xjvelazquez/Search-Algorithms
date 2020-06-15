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
        self.final_cost = 0 #Make sure to update this value at the end of UCS and Astar
        self.search(type)
        self.seen = {}
        self.G = {self.start: 0}
        self.count = 0
    def search(self, type):
        self.finished = False
        self.failed = False
        self.type = type
        self.previous = {}
        if self.type == "dfs":
            self.frontier = [self.start]
            self.explored = []
        elif self.type == "bfs":
            self.frontier = [self.start]
            self.explored = []
        elif self.type == "ucs":
            self.frontier = [(0, self.start)]
            self.explored = []
        elif self.type == "astar":
            self.frontier = [(0, self.start)]
            self.explored = []
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
    #DFS
    def dfs_step(self):
        if not self.frontier:
            self.failed = True
            print("no path")
            return
        current = self.frontier.pop()
        self.grid.nodes[current].checked = True
        self.grid.nodes[current].frontier = False
        self.explored.append(current)
        children = [(current[0]+a[0], current[1]+a[1]) for a in ACTIONS]
        for node in children:
            if node in self.explored or node in self.frontier:
                continue
            if node[0] in range(self.grid.row_range) and node[1] in range(self.grid.col_range):
                if not self.grid.nodes[node].puddle:
                    self.previous[node] = current
                    if node == self.goal:
                        self.finished = True
                    else:
                        self.frontier.append(node)
                        self.grid.nodes[node].frontier = True
    #Implement BFS here
    def bfs_step(self):
        if not self.frontier:
            self.failed = True
            print("no path")
            return
        current = self.frontier.pop(0)
        self.grid.nodes[current].checked = True
        self.grid.nodes[current].frontier = False
        self.explored.append(current)
        children = [(current[0]+a[0], current[1]+a[1]) for a in ACTIONS]
        for node in children:
            if node in self.explored or node in self.frontier:
                continue
            if node[0] in range(self.grid.row_range) and node[1] in range(self.grid.col_range):
                if not self.grid.nodes[node].puddle:
                    self.previous[node] = current
                    if node == self.goal:
                        self.finished = True
                    else:
                        self.frontier.append(node)
                        self.grid.nodes[node].frontier = True
    #Implement UCS here
    def ucs_step(self):
        if self.count == 0:
            self.seen = {}
            self.frontier = [(0, self.start)]
            heappush(self.frontier, (0, self.start))
        if not self.frontier:
            self.failed = True
            print("no path")
            self.count = 0
            return
        current = heappop(self.frontier)
        current_cost = current[0]
        self.grid.nodes[current[1]].checked = True
        self.grid.nodes[current[1]].frontier = False
        self.explored.append(current[1])
        if current[1] == self.goal:
            self.finished = True
            self.final_cost = current[0]
            self.count = 0
            print('Final cost',self.final_cost)
            return
        children = [(current[1][0]+a[0], current[1][1]+a[1]) for a in ACTIONS]
        for node in children:
            if node[0] in range(self.grid.row_range) and node[1] in range(self.grid.col_range):
                node_cost = self.grid.nodes[node].cost()
                if not self.grid.nodes[node].puddle:
                    if not (node in self.explored or node in self.seen):
                        heappush(self.frontier, (current_cost + node_cost, node))
                        self.previous[node] = current[1]
                        self.grid.nodes[node].frontier = True
                        self.seen[node] = current_cost + node_cost
                    elif node in self.explored:
                        continue
                    elif self.seen[node] > current_cost + node_cost:
                        self.frontier.remove((self.seen[node], node))
                        heappush(self.frontier, (current_cost + node_cost, node))
                        self.seen[node] = current_cost + node_cost
        self.count += 1
                    
    #Implement Astar here
    def astar_step(self):
        # If new run through of Astar 
        if self.count == 0:
            self.seen = {}
            self.G = {self.start: 0}
            heappush(self.frontier, (0, self.start))  
        # If frontier is empty      
        if not self.frontier:
            self.failed = True
            self.count = 0
            print("no path a")
            return
        current = heappop(self.frontier)
        current_cost = current[0]
        self.grid.nodes[current[1]].checked = True
        self.grid.nodes[current[1]].frontier = False
        self.explored.append(current[1])
        # If current node is the goal node
        if current[1] == self.goal:
            self.finished = True
            self.final_cost = current[0]
            self.count = 0
            print('Final cost', self.final_cost)
            return
        children = [(current[1][0]+a[0], current[1][1]+a[1]) for a in ACTIONS]
        for node in children:
            if node[0] in range(self.grid.row_range) and node[1] in range(self.grid.col_range):
                node_cost = self.grid.nodes[node].cost()
                h_cost = abs(node[0] - self.goal[0]) + abs(node[1] - self.goal[1])
                if not self.grid.nodes[node].puddle:
                    if not (node in self.explored or node in self.seen):
                        heappush(self.frontier, (self.G[current[1]] + node_cost + h_cost, node))
                        self.previous[node] = current[1]
                        self.grid.nodes[node].frontier = True
                        self.seen[node] = (self.G[current[1]] + node_cost + h_cost)
                        self.G[node] = self.G[current[1]] + node_cost
                    if node in self.explored:
                        continue
                    if self.seen[node] > (self.G[current[1]] + node_cost + h_cost):
                        self.frontier.remove((self.seen[node], node))
                        heappush(self.frontier, (self.G[current[1]] + node_cost + h_cost, node))
                        self.seen[node] = (self.G[current[1]] + node_cost + h_cost)
                        self.G[node] = self.G[current[1]] + node_cost
        self.count += 1