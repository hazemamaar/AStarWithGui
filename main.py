import math
import random
from queue import PriorityQueue
import time  # av best worst
import pygame

WIDTH = 800
WIN = pygame.display.set_mode((WIDTH, WIDTH))
pygame.display.set_caption("A* Path Planning")

fringe = list()
f_score_cost = list()
g_score_cost = list()
heuristic_cost = list()
current_node = list()
neighbor_node = list()
times = list()
final_path = list()

# Colors We are used in this program(RGB)
MAROON = (128, 0, 0)
BLUE = (0, 191, 255)
WHITE = (255, 250, 250)
BLACK = (50, 50, 50)
VIOLET = (238, 130, 238)
GOLD = (218, 165, 32)
SILVER = (192, 192, 192)
FUCHSIA = (255, 0, 255)


# Each instance of this class represents a spot or a cell in the grid.
class Node:
    # row , col , width and total_row . These parameters define the position and size of the spot in the grid.
    def __init__(self, row, col, width, total_rows):
        self.row = row
        self.col = col
        self.x = row * width
        self.y = col * width
        self.color = WHITE
        self.neighbors = []
        self.width = width
        self.total_rows = total_rows

    # returns the row and column of the spot.
    def get_pos(self):
        return self.row, self.col

    # check the color of the spot and return True if it matches the corresponding color.
    def is_closed(self):
        return self.color == MAROON

    def is_open(self):
        return self.color == BLUE

    def is_barrier(self):
        return self.color == BLACK

    def is_start(self):
        return self.color == GOLD

    def is_end(self):
        return self.color == FUCHSIA

    # change the color of the spot to the corresponding color.
    def reset(self):
        self.color = WHITE

    def make_start(self):
        self.color = GOLD

    def make_closed(self):
        fringe.remove(self.get_pos())
        self.color = MAROON

    def make_open(self):
        fringe.append(self.get_pos())
        self.color = BLUE

    def make_barrier(self):
        self.color = BLACK

    def make_end(self):
        self.color = FUCHSIA

    def make_path(self):
        final_path.append(self.get_pos())
        self.color = VIOLET

    # This method is used to draw the node on the Pygame window.
    def draw(self, win):
        pygame.draw.rect(win, self.color, (self.x, self.y, self.width, self.width))

    # this method takes the grid as a parameter and updates the neighbors list of the spot based on its position and the
    # positions of its adjacent spots.
    def update_neighbors(self, grid):
        self.neighbors = []
        if self.row < self.total_rows - 1:
            # DOWN
            if not grid[self.row + 1][self.col].is_barrier():
                self.neighbors.append(grid[self.row + 1][self.col])

            # DOWN-RIGHT
            if self.col < self.total_rows - 1 and not grid[self.row + 1][self.col + 1].is_barrier():
                self.neighbors.append(grid[self.row + 1][self.col + 1])

            # DOWN-LEFT
            if self.col > 0 and not grid[self.row + 1][self.col - 1].is_barrier():
                self.neighbors.append(grid[self.row + 1][self.col - 1])

        if self.row > 0:
            # UP
            if not grid[self.row - 1][self.col].is_barrier():
                self.neighbors.append(grid[self.row - 1][self.col])

            # UP-RIGHT
            if self.col < self.total_rows - 1 and not grid[self.row - 1][self.col + 1].is_barrier():
                self.neighbors.append(grid[self.row - 1][self.col + 1])

            # UP-LEFT
            if self.col > 0 and not grid[self.row - 1][self.col - 1].is_barrier():
                self.neighbors.append(grid[self.row - 1][self.col - 1])

        if self.col < self.total_rows - 1:
            # RIGHT
            if not grid[self.row][self.col + 1].is_barrier():
                self.neighbors.append(grid[self.row][self.col + 1])

        if self.col > 0:
            # LEFT
            if not grid[self.row][self.col - 1].is_barrier():
                self.neighbors.append(grid[self.row][self.col - 1])

    # compare two instances of the Note class.
    def __lt__(self, other):
        return False


# Heuristic
def heuristic(p1, p2):
    x1, y1 = p1
    x2, y2 = p2
    dx = abs(x1 - x2)
    dy = abs(y1 - y2)
    return math.sqrt(dx * dx + dy * dy)


# reconstructing the optimal path from the starting point to the end
def reconstruct_path(came_from, current, draw):
    while current in came_from:
        current = came_from[current]
        current.make_path()
        draw()


# a star Algorithm
def astar(draw, grid, start, end):
    count = 0  # Initialize a counter to keep track of the order in which nodes are added to the open set.
    open_set = PriorityQueue()  # Create a priority queue to store nodes that are candidates for exploration.
    open_set.put((0, count, start))  # Add the start node to the open set with a priority of 0.
    came_from = {}  # Initialize an empty dictionary to store the parent node for each node in the grid.
    g_score = {spot: float("inf") for row in grid for spot in row}  # Initialize g-scores of all nodes to infinity.
    g_score[start] = 0  # Set the g-score of the start node to 0.
    f_score = {spot: float("inf") for row in grid for spot in row}  # Initialize f-scores of all nodes to infinity.
    # fscore = h+g
    f_score[start] = heuristic(start.get_pos(), end.get_pos())  # Calculate the f-score of the start node.

    open_set_hash = {start}  # Initialize a set to keep track of nodes present in the open set.

    while not open_set.empty():
        for event in pygame.event.get():  # Check for any events (e.g., window close) during the algorithm's execution.
            if event.type == pygame.QUIT:
                pygame.quit()

        current = open_set.get()[2]  # Get the node with the lowest f-score from the open set.
        open_set_hash.remove(current)

        if current == end:
            reconstruct_path(came_from, end, draw)  # Reconstruct the shortest path if the end node is reached.
            end.make_end()
            return True

        for neighbor in current.neighbors:
            if neighbor.is_barrier():  # Skip neighbors that are barriers.
                continue

            if current.row != neighbor.row and current.col != neighbor.col:
                # Diagonal move cost (√2)
                temp_g_score = g_score[current] + math.sqrt(2)
            else:
                # Horizontal or vertical move cost (1)
                temp_g_score = g_score[current] + 1

            if temp_g_score < g_score[neighbor]:
                came_from[neighbor] = current  # Update the parent node for the neighbor.
                g_score[neighbor] = temp_g_score  # Update the g-score of the neighbor.
                f_score[neighbor] = temp_g_score + heuristic(neighbor.get_pos(), end.get_pos())
                current_node.append(current.get_pos())
                neighbor_node.append(neighbor.get_pos())
                f_score_cost.append(f_score[neighbor])
                g_score_cost.append(temp_g_score)
                heuristic_cost.append(heuristic(neighbor.get_pos(), end.get_pos()))
                if neighbor not in open_set_hash:
                    count += 1
                    open_set.put((f_score[neighbor], count, neighbor))  # Add the neighbor to the open set.
                    open_set_hash.add(neighbor)
                    neighbor.make_open()

        if end.get_pos() in fringe:
            fringe.remove(end.get_pos())
        draw()  # Update the visualization of the grid.
        if current != start:
            current.make_closed()
    return False  # Return False if no path to the end node is found.


def make_grid(rows, width):
    grid = []  # Initialize an empty list to store the grid
    gap = width // rows  # Calculate the size of each cell in the grid

    for i in range(rows):
        grid.append([])  # Append an empty list to represent a new row in the grid

        for j in range(rows):
            spot = Node(i, j, gap, rows)  # Create a new Node object for each cell
            grid[i].append(spot)  # Add the node to the current row in the grid

    return grid  # Return the generated grid


def draw_grid(win, rows, width):
    gap = width // rows  # Calculate the size of each grid cell based on the number of rows and the window width.

    # Draw horizontal grid lines
    for i in range(rows):
        pygame.draw.line(win, SILVER, (0, i * gap),
                         (width, i * gap))  # Draw a horizontal line from left to right at each row.

        # Draw vertical grid lines
        for j in range(rows):
            pygame.draw.line(win, SILVER, (j * gap, 0),
                             (j * gap, width))  # Draw a vertical line from top to bottom at each column.


# This function is responsible for drawing the grid and updating the window
def draw(win, grid, rows, width):
    win.fill(WHITE)  # Fill the entire window with white color.

    for row in grid:
        for spot in row:
            spot.draw(win)  # Draw each spot (node) in the grid on the window.

    draw_grid(win, rows, width)  # Draw the grid lines on the window.
    pygame.display.update()  # Update the window to show the changes.


# This function is used to convert the mouse click position to the corresponding row and column indices in the grid.
def get_clicked_pos(pos, rows, width):
    gap = width // rows  # Calculate the size of each spot (node) in the grid.

    y, x = pos  # Retrieve the x and y coordinates of the mouse click.

    row = y // gap  # Calculate the row index based on the y coordinate and spot size.
    col = x // gap  # Calculate the column index based on the x coordinate and spot size.

    return row, col  # Return the row and column indices of the clicked spot.


def reset_grid(grid):
    for row in grid:
        for spot in row:
            spot.reset()


# def main(win, width):
#     ROWS = 10  # Number of rows in the grid
#     grid = make_grid(ROWS, width)  # Create the grid
#
#     start = None  # Variable to store the start spot
#     end = None  # Variable to store the end spot
#
#     obstacle_count = 0  # Counter for generated obstacles
#     run = True
#     while run:
#         draw(win, grid, ROWS, width)  # Draw the grid
#
#         for event in pygame.event.get():
#             if event.type == pygame.QUIT:
#                 run = False
#
#             if pygame.mouse.get_pressed()[0]:  # LEFT mouse button clicked
#                 pos = pygame.mouse.get_pos()
#                 row, col = get_clicked_pos(pos, ROWS, width)
#                 spot = grid[row][col]
#
#                 if not start and spot != end:
#                     start = spot
#                     start.make_start()
#
#                 elif not end and spot != start:
#                     end = spot
#                     end.make_end()
#
#                 elif spot != end and spot != start:
#                     if not spot.is_barrier():  # Check if the cell is not already a barrier
#                         spot.make_barrier()
#                         obstacle_count += 1
#
#             elif pygame.mouse.get_pressed()[2]:  # RIGHT mouse button clicked
#                 pos = pygame.mouse.get_pos()
#                 row, col = get_clicked_pos(pos, ROWS, width)
#                 spot = grid[row][col]
#                 spot.reset()
#                 if spot == start:
#                     start = None
#                 elif spot == end:
#                     end = None
#
#             if event.type == pygame.KEYDOWN:
#                 if event.key == pygame.K_SPACE and start and end:
#                     for row in grid:
#                         for spot in row:
#                             spot.update_neighbors(grid)
#
#                     astar(lambda: draw(win, grid, ROWS, width), grid, start, end)  # Run A* algorithm
#                 for heuristic, g_cost, f_cost, current, neighbor in zip(heuristic_cost, g_score_cost,
#                                                                         f_score_cost, current_node,
#                                                                         neighbor_node):
#                     print("current node: ", current)
#                     print("neighbor: ", neighbor)
#                     print("FScore of current neighbor:", f_cost, "=", g_cost, "+",
#                           heuristic)
#                     print("\n   -------------------")
#                 if len(final_path) > 0:
#                     print("                final path")
#                     for final in reversed(final_path):
#                         print(final)
#                 else:
#                     print("No Path founded")
#
#                 if len(fringe) > 0:
#                     print("                 Fringe")
#                     for fringe_score in fringe:
#                         print(fringe_score)
#                 else:
#                     print("No Fringe founded")
#
#                 if event.key == pygame.K_c:
#                     start = None
#                     end = None
#                     obstacle_count = 0
#                     grid = make_grid(ROWS, width)  # Reset the grid
#     fringe.clear()
#     final_path.clear()
#     heuristic_cost.clear()
#     g_score_cost.clear()
#     current_node.clear()
#     neighbor_node.clear()
#     f_score_cost.clear()
#     pygame.quit()


# Main function
def main(win, width):
    ROWS = 10  # Number of rows in the grid
    grid = make_grid(ROWS, width)  # Create the grid

    start = grid[0][0]  # Variable to store the start spot
    end = grid[9][9]  # Variable to store the end spot
    start.make_start()
    end.make_end()
    for i in range(20):

        draw(win, grid, ROWS, width)  # Draw the grid
        for row in grid:
            for spot in row:
                spot.update_neighbors(grid)
        obstacle_count = 0  # Counter for generated obstacles
        while obstacle_count < 20:
            row = random.randint(0, ROWS - 1)
            col = random.randint(0, ROWS - 1)
            spot = grid[row][col]
            if not spot.is_barrier() and spot != start and spot != end:  # Check if the cell is not already a barrier
                spot.make_barrier()
                obstacle_count += 1
        start_time = time.time()
        astar(lambda: draw(win, grid, ROWS, width), grid, start, end)  # Run A* algorithm
        end_time = time.time()
        times.append(end_time - start_time)
        if i == 0:
            print("            50% obstacles")
        print("                        test", i + 1)
        for heuristic, g_cost, f_cost, current, neighbor in zip(heuristic_cost, g_score_cost,
                                                                f_score_cost, current_node,
                                                                neighbor_node):
            print("current node: ", current)
            print("neighbor: ", neighbor)
            print("FScore of current neighbor:", f_cost, "=", g_cost, "+",
                  heuristic)
            print("\n   -------------------")
        if len(final_path) > 0:
            print("                final path")
            for final in reversed(final_path):
                print(final)
        else:
            print("No Path founded")

        if len(fringe) > 0:
            print("                 Fringe")
            for fringe_score in fringe:
                print(fringe_score)
        else:
            print("No Fringe founded")
        pygame.display.update()
        reset_grid(grid)
        fringe.clear()
        final_path.clear()
        heuristic_cost.clear()
        g_score_cost.clear()
        current_node.clear()
        neighbor_node.clear()
        f_score_cost.clear()

    pygame.quit()
    average_time = sum(times) / len(times)
    best_time = min(times)
    worst_time = max(times)
    print("average time is:", average_time)
    print("best time is:", best_time)
    print("worst time is:", worst_time)


main(WIN, WIDTH)
