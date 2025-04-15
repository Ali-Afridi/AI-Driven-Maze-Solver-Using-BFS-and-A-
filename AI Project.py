import heapq
import random
import time
import copy
import tkinter as tk

cell_width = 20
cell_height = 20

# Class representing the maze environment
class MazeEnvironment:
    def __init__(self, maze):
        self.maze = maze
        self.rows = len(maze)
        self.cols = len(maze[0])
        self.start = (0, 0)
        self.goal = (self.rows - 1, self.cols - 1)
        self.obstacles = set()
        self.user_obstacles = set()

    def add_obstacle(self, position):
        self.user_obstacles.add(position)

    def remove_obstacle(self, position):
        if position in self.user_obstacles:
            self.user_obstacles.remove(position)

    def is_valid_move(self, position):
        return 0 <= position[0] < self.rows and 0 <= position[1] < self.cols \
               and position not in self.obstacles and position not in self.user_obstacles \
               and self.maze[position[0]][position[1]] != '#'

    def get_neighbors(self, position):
        neighbors = [(position[0] + 1, position[1]), (position[0] - 1, position[1]),
                     (position[0], position[1] + 1), (position[0], position[1] - 1)]
        return [neighbor for neighbor in neighbors if self.is_valid_move(neighbor)]

# Function to generate a random maze
def generate_random_maze(rows, cols, obstacle_density):
    maze = [[' ' for _ in range(cols)] for _ in range(rows)]
    for i in range(rows):
        for j in range(cols):
            if random.random() < obstacle_density:
                maze[i][j] = '#'
    return maze

# Function to visualize the maze on the canvas
def visualize_maze(canvas, maze_env, start=None, goal=None, path=[]):
    canvas.delete("all")  # Clear canvas
    maze = maze_env.maze

    # Draw maze on canvas
    for i in range(len(maze)):
        for j in range(len(maze[0])):
            if maze[i][j] == '#':  # Check for '#' to fill with blue
                canvas.create_rectangle(j * cell_width, i * cell_height, (j + 1) * cell_width,
                                        (i + 1) * cell_height,
                                        fill='blue')
            elif maze[i][j] == ' ':  # Empty space
                canvas.create_rectangle(j * cell_width, i * cell_height, (j + 1) * cell_width,
                                        (i + 1) * cell_height,
                                        fill='white')
            elif maze[i][j] == 'S':  # Start point
                canvas.create_rectangle(j * cell_width, i * cell_height, (j + 1) * cell_width,
                                        (i + 1) * cell_height,
                                        fill='green')
            elif maze[i][j] == 'G':  # Goal point
                canvas.create_rectangle(j * cell_width, i * cell_height, (j + 1) * cell_width,
                                        (i + 1) * cell_height,
                                        fill='red')
            else:  # User-defined obstacle
                canvas.create_rectangle(j * cell_width, i * cell_height, (j + 1) * cell_width,
                                        (i + 1) * cell_height,
                                        fill='black')

    # Draw path on canvas
    for pos in path:
        if pos not in maze_env.obstacles:  # Don't draw obstacles in yellow
            canvas.create_rectangle(pos[1] * cell_width, pos[0] * cell_height, (pos[1] + 1) * cell_width,
                                    (pos[0] + 1) * cell_height, fill='yellow')

    # Highlight start and goal points
    if start:
        canvas.create_rectangle(start[1] * cell_width, start[0] * cell_height, (start[1] + 1) * cell_width,
                                (start[0] + 1) * cell_height, fill='green')
    if goal:
        canvas.create_rectangle(goal[1] * cell_width, goal[0] * cell_height, (goal[1] + 1) * cell_width,
                                (goal[0] + 1) * cell_height, fill='red')

# Class representing an uninformed search agent
class UninformedSearchAgent:
    def __init__(self, environment):
        self.environment = environment
        self.visited = set()
        self.path = []

    def bfs_search(self):
        queue = [(self.environment.start, [])]

        while queue:
            current, path = queue.pop(0)

            if current == self.environment.goal:
                self.path = path + [current]
                return True

            if current not in self.visited:
                self.visited.add(current)
                neighbors = self.environment.get_neighbors(current)
                for neighbor in neighbors:
                    queue.append((neighbor, path + [current]))

        return False

# Class representing an informed search agent
class InformedSearchAgent:
    def __init__(self, environment):
        self.environment = environment
        self.visited = set()
        self.path = []

    def heuristic(self, position):
        return abs(position[0] - self.environment.goal[0]) + abs(position[1] - self.environment.goal[1])

    def astar_search(self):
        heap = [(self.heuristic(self.environment.start), self.environment.start, [])]
        heapq.heapify(heap)

        while heap:
            _, current, path = heapq.heappop(heap)

            if current == self.environment.goal:
                self.path = path + [current]
                return True

            if current not in self.visited:
                self.visited.add(current)
                neighbors = self.environment.get_neighbors(current)
                for neighbor in neighbors:
                    heapq.heappush(heap, (self.heuristic(neighbor) + len(path) + 1, neighbor, path + [current]))

        return False

# Function to mark an obstacle on the canvas
def mark_obstacle(event):
    col = event.x // cell_width
    row = event.y // cell_height
    maze_env.add_obstacle((row, col))
    visualize_maze(canvas, maze_env)

# Function to find and visualize the path
# Function to find and animate the path
def find_path():
    global status_label

    bfs_agent = UninformedSearchAgent(maze_env)
    bfs_found = bfs_agent.bfs_search()

    astar_agent = InformedSearchAgent(maze_env)
    astar_found = astar_agent.astar_search()

    if bfs_found or astar_found:
        status_label.config(text="Path found.")
        if bfs_found:
            path = bfs_agent.path
        else:
            path = astar_agent.path

        # Animate the path
        for pos in path[:-1]:
            visualize_maze(canvas, maze_env, start=maze_env.start, goal=maze_env.goal, path=[pos])
            time.sleep(0.5)
            root.update()

        # Show the complete path
        visualize_maze(canvas, maze_env, start=maze_env.start, goal=maze_env.goal, path=path)
    else:
        status_label.config(text="No path found.")

    # Update the GUI after finding the path
    root.update()

# Main tkinter window
root = tk.Tk()
root.title("Maze Visualization")

rows = 15
cols = 15
obstacle_density = 0.3

canvas = tk.Canvas(root, width=cols * 20, height=rows * 20, bg='white')
canvas.pack()

maze_env = MazeEnvironment(generate_random_maze(rows, cols, obstacle_density))

# Bind left mouse click to mark obstacles
canvas.bind("<Button-1>", mark_obstacle)

# Bind right mouse click to find path
canvas.bind("<Button-3>", lambda event: find_path())

visualize_maze(canvas, maze_env, start=maze_env.start, goal=maze_env.goal)

status_label = tk.Label(root, text="", pady=10)
status_label.pack()

root.mainloop()