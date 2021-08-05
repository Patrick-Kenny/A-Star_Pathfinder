# A* Pathfinding and Visualization
# Developed by Patrick Kenny

from types import LambdaType
import sys
import os
import pygame
from queue import PriorityQueue
from enum import Enum

ROW_LIMIT = 100 # Size limit for maze height
COL_LIMIT = 100 # Size limit for maze width

# List of colors to display
RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
ORANGE = (255, 165, 0)
GRAY = (128, 128, 128)
TURQUOISE = (64, 224, 208)

class NodeType(Enum):
    CLOSED = "C"
    OPEN = "O"
    UNVISITED = "."
    OBSTACLE = "#"
    PATH = "P"
    START = "S"
    GOAL = "G"

class Node:
    def __init__(self, row, col, nodeWidth, totalRows, totalCols):
        self.row = row
        self.col = col
        self.width = nodeWidth
        self.x = row * self.width
        self.y = col * self.width
        self.color = WHITE
        self.neighbors = []
        self.totalRows = totalRows
        self.totalCols = totalCols
        self.type = NodeType.UNVISITED
    
    def get_position(self):
        return self.row, self.col

    def is_obstacle(self):
        return self.type == NodeType.OBSTACLE

    def is_start(self):
        return self.type == NodeType.START

    def is_goal(self):
        return self.type == NodeType.GOAL

    def set_start(self):
        self.type = NodeType.START
        self.color = ORANGE

    def set_closed(self):
        self.type = NodeType.CLOSED
        self.color = RED

    def set_open(self):
        self.type = NodeType.OPEN
        self.color = BLUE

    def set_obstacle(self):
        self.type = NodeType.OBSTACLE
        self.color = BLACK
    
    def set_goal(self):
        self.type = NodeType.GOAL
        self.color = TURQUOISE

    def set_path(self):
        self.type = NodeType.PATH
        self.color = GREEN

    def draw(self, window):
        pygame.draw.rect(window, self.color, (self.y, self.x, self.width, self.width))

    def update_neighbors(self, level):
        # Checks each neighbor to see if not an obstacle
        
        self.neighbors = []
        try:
            if self.row > 0 and not level[self.row - 1][self.col].is_obstacle(): # Looks Up
                self.neighbors.append(level[self.row - 1][self.col])
            if self.row < self.totalRows - 1 and not level[self.row + 1][self.col].is_obstacle(): # Looks Down
                self.neighbors.append(level[self.row + 1][self.col])
            if self.col > 0 and not level[self.row][self.col - 1].is_obstacle(): # Looks Left
                self.neighbors.append(level[self.row][self.col - 1])
            if self.col < self.totalCols - 1 and not level[self.row][self.col + 1].is_obstacle(): # Looks Right
                self.neighbors.append(level[self.row][self.col + 1])
        except:
            pass

def h(node1, node2):
    # Gets heuristic value, showing the (manhattan) distance between current and goal nodes
    x1, y1 = node1
    x2, y2 = node2
    return abs(x2 - x1) + abs(y2 - y1)

def reconstruct_path(cameFrom, current, draw):
    # Reconstructs the optimal path taken from Robot to Goal
    while current in cameFrom:
        current = cameFrom[current]
        if not current.is_start() and not current.is_goal():
            current.set_path()
        draw()

def find_path(draw, level, start, goal):
    # Searches through nodes to find best path from Robot to Goal
    # Uses A* search algorithm, f(x) = g(x) + h(x)
    # h(x) being the (Manhattan) distance from the current node to the goal node
    # g(x) being the path length from the start node to the current node
    # f(x) being the total score of the two

    count = 0

    visualize = True # Determines if the entire process should display

    openNodes = PriorityQueue()
    openNodes.put((0, count, start))

    came_from = {}

    gScore = {node: float("inf") for row in level for node in row}
    gScore[start] = 0

    fScore = {node: float("inf") for row in level for node in row}
    fScore[start] = h(start.get_position(), goal.get_position())

    openNodesHash = {start}

    while not openNodes.empty():
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                exit()

            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_SPACE:
                    visualize = False # Skips the visualization of the search

                if event.key == pygame.K_ESCAPE:
                    pygame.quit()
                    exit()

        current = openNodes.get()[2]
        openNodesHash.remove(current)

        if current == goal:
            current.set_goal()
            reconstruct_path(came_from, goal, draw)
            return True
        
        for neighbor in current.neighbors:
            neighborGScore = gScore[current] + 1

            # Ensures that the neighbor's calculated gScore is less than its current gScore
            # (i.e. the current path taken to it is the shortest path to it, otherwise skip node)
            if neighborGScore < gScore[neighbor]:
                came_from[neighbor] = current
                gScore[neighbor] = neighborGScore
                fScore[neighbor] = neighborGScore + h(neighbor.get_position(), goal.get_position())
                
                if neighbor not in openNodesHash:
                    count += 1
                    openNodes.put((fScore[neighbor], count, neighbor))
                    openNodesHash.add(neighbor)
                    neighbor.set_open()
        if visualize:
            draw()

        if current != start:
            current.set_closed()

    print("No path found.")
    return False

def make_level(drawData, size, filename):
    # Reads specified file and constructs the map

    level = []
    file = open(filename, "r")

    start = None
    goal = None

    for i, line in enumerate(file):
        level.append([])
        for j, char in enumerate(line):
            node = Node(i, j, drawData["gap"], size["rows"], size["cols"])

            if char == NodeType.OBSTACLE.value:
                node.set_obstacle()
            elif char == NodeType.START.value:
                node.set_start()
                start = node
            elif char == NodeType.GOAL.value:
                node.set_goal()
                goal = node
            elif char == NodeType.UNVISITED.value or char == "\n":
                pass
            else:
                print("ERROR: Invalid character(s) in map file: {}".format(char), sys.stderr)
                exit()
            if j > COL_LIMIT:
                print("ERROR: Invalid maze width. (Limit: 1-{})".format(COL_LIMIT), file = sys.stderr)
                exit()

            level[i].append(node)

        if i > ROW_LIMIT:
            print("ERROR: Invalid maze height. (Limit: 1-{})".format(ROW_LIMIT), file = sys.stderr)
            exit()

    size["cols"] = j+1
    size["rows"] = i+1
    drawData["width"] = drawData["gap"] * size["cols"]
    drawData["height"] = drawData["gap"] * size["rows"]

    for row in level:
        for node in row:
            node.totalRows = size["rows"]
            node.totalCols = size["cols"]

    file.close()
    return level, start, goal

def draw_grid(drawData, mazeSize, window):
    # Adds grid lines to visualization
    for i in range(mazeSize["rows"]+1):
        pygame.draw.line(window, GRAY, (0, i * drawData["gap"]), (drawData["width"], i * drawData["gap"]))
    for j in range(mazeSize["cols"]+1):
        pygame.draw.line(window, GRAY, (j * drawData["gap"], 0), (j * drawData["gap"], drawData["height"]))

def draw(window, level, drawData, mazeSize):
    # Draws the map
    window.fill(WHITE)

    for row in level:
        for node in row:
            node.draw(window)

    draw_grid(drawData, mazeSize, window)
    pygame.display.update()

def main(filename):

    DRAW_DATA = {"width": 0, "height": 0, "gap": 10}
    MAZE_SIZE = {"rows": 0, "cols": 0}

    start = None
    goal = None

    running = False
    level, start, goal = make_level(DRAW_DATA, MAZE_SIZE, filename)

    WINDOW = pygame.display.set_mode((DRAW_DATA["width"], DRAW_DATA["height"]))
    pygame.display.set_caption("A* Path Finding Algorithm")

    draw(WINDOW, level, DRAW_DATA, MAZE_SIZE)

    print("\nPress Space to start. Press ESC to exit.")
    print("While running, press Space to skip search visualization.")
    print("When completed, press Space again to reset.")
    
    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                exit()

            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_SPACE:
                    if running:
                        level = []
                        level, start, goal = make_level(DRAW_DATA, MAZE_SIZE, filename)
                        draw(WINDOW, level, DRAW_DATA, MAZE_SIZE)
                    else:
                        running = True
                    for row in level:
                        for node in row:
                            node.update_neighbors(level)

                    find_path(lambda: draw(WINDOW, level, DRAW_DATA, MAZE_SIZE), level, start, goal)
                
                if event.key == pygame.K_ESCAPE:
                    pygame.quit()
                    exit()

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: {} <FILENAME>".format(os.path.basename(__file__)))
        exit()
    filename = sys.argv[1]
    main(filename)