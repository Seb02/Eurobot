import math
import numpy as np
import matplotlib.pyplot as plt
import heapq
import random
from main import Spline


class Grid:
    def __init__(self, width=300, height=200, square_size=1, robot_pos=(0, 0), robot_angle=0, goal=(200, 200), robot_size=1):
        self.width = width
        self.height = height
        self.square_size = square_size
        self.obstacles = set()
        self.robot_pos = robot_pos
        self.robot_angle = robot_angle
        self.goal = goal
        self.robot_size = robot_size

    def add_point(self, x, y, point_type):
        if x > self.width or y > self.height or x < 0 or y <0:
            pass
        point = (x, y)
        if point_type.lower() == 'avoid':
            self.obstacles.add(point)
        elif point_type.lower() == 'path':
            self.path.add(point)
        else:
            raise ValueError('Invalid point type')

    def move_robot(self, x, y, angle):
        if x > self.width or y > self.height or x < 0 or y <0:
            raise ValueError('Point lies outside the grid')
        self.robot_pos = (x, y)
        self.robot_angle = angle

    def angle_distance_to_point(self, distances_angles):
        points = []
        for distance, angle in distances_angles:
            x = self.robot_pos[0] + distance * math.cos(angle + self.robot_angle)
            y = self.robot_pos[1] + distance * math.sin(angle + self.robot_angle)
            if x > self.width or y > self.height or x < 0 or y <0:
                continue
            points.append((x, y))
        return points
    
    def bulk_add_points(self, points, point_type):
        for x, y in points:
            self.add_point(x, y, point_type)

    def refresh_grid(self, points):
        self.obstacles = set()
        self.bulk_add_points(points, 'avoid')
    
    def convert_to_cell_grid_object(self):
        grid = CellGrid(self.width//self.square_size, self.height//self.square_size)
        for x, y in self.obstacles:
            grid.add_obstacle(x//self.square_size, y//self.square_size)
        grid.set_start(self.robot_pos[0]//self.square_size, self.robot_pos[1]//self.square_size)
        grid.set_end(self.goal[0]//self.square_size, self.goal[1]//self.square_size)
        return grid
    
    def bulk_convert_to_normal_coordinates(self, coordinates):
        return [(x*self.square_size, y*self.square_size) for x, y in coordinates]
    
    def convert_to_normal_coordinates(self, coordinates):
        return (coordinates[0]*self.square_size, coordinates[1]*self.square_size)



class CellGrid:
    def __init__(self, width, height, robot_size=1):
        self.width = width
        self.height = height
        self.grid = np.zeros((height, width))
        self.start = None
        self.end = None
        self.robot_size = robot_size
        self.path = None

    def add_obstacle(self, x, y):
        if 0 <= x < self.width and 0 <= y < self.height:
            x = int(x)
            y = int(y)
            self.grid[y, x] = 1

    def set_start(self, x, y):
        self.start = (x, y)
        self.grid[y, x] = 2

    def set_end(self, x, y):
        self.end = (x, y)
        self.grid[y, x] = 3

    def is_obstacle(self, x, y):
        return self.grid[y, x] == 1

    def is_valid_position(self, x, y):
        if not (0 <= x < self.width and 0 <= y < self.height):
            return False
        for dy in range(self.robot_size):
            for dx in range(self.robot_size):
                nx, ny = x + dx, y + dy
                if not (0 <= nx < self.width and 0 <= ny < self.height) or self.is_obstacle(nx, ny):
                    return False
        return True

    def get_close_obstacle_points(self, distance_threshold):
        close_points = set()
        for y in range(self.height):
            for x in range(self.width):
                if self.grid[y, x] != 1: 
                    for dy in range(-distance_threshold, distance_threshold + 1):
                        for dx in range(-distance_threshold, distance_threshold + 1):
                            nx, ny = x + dx, y + dy
                            if 0 <= nx < self.width and 0 <= ny < self.height and self.is_obstacle(nx, ny):
                                close_points.add((x, y))
                                break 
        return list(close_points)
    
    def get_waypoints(self, distance_threshold):
        close_obstacle_points = self.get_close_obstacle_points(distance_threshold)
        waypoints = []
        for point in self.path:
            if point in close_obstacle_points:
                waypoints.append(point)
        return waypoints
    
    def plot_grid(self, path):
        fig, ax = plt.subplots()
        ax.imshow(self.grid, cmap=plt.cm.terrain)
        ax.scatter([p[0] for p in path], [p[1] for p in path], marker='o', color='red', label='Path')
        ax.scatter(*self.start, marker='*', color='green', label='Start')
        ax.scatter(*self.end, marker='*', color='blue', label='End')
        obstacles = np.argwhere(self.grid == 1)
        ax.scatter(obstacles[:, 1], obstacles[:, 0], marker='x', color='black', label='Obstacles')
        ax.legend()
        ax.set_title('A* Pathfinding with Diagonal Movement')
        ax.set_xlabel('X Coordinate')
        ax.set_ylabel('Y Coordinate')
        plt.show()
    
    def get_obstacles(self):
        """ Get a list of sets of coordinates of obstacles
        """
        obstacles = []
        for y in range(self.height):
            for x in range(self.width):
                if self.grid[y, x] == 1:
                    obstacles.append((x, y))
        return obstacles


class Astar:

    def __init__(self):
        pass

    def euclidean_distance(self, a, b):
        return np.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)

    def a_star_search_diagonal(self, grid, start, end):
        frontier = []
        heapq.heappush(frontier, (0, start))
        came_from = {start: None}
        cost_so_far = {start: 0}

        while frontier:
            _, current = heapq.heappop(frontier)


            if current == end:
                break

            for dx, dy in [(1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (-1, -1), (1, -1), (-1, 1)]:
                next_node = (current[0] + dx, current[1] + dy)
                if grid.is_valid_position(next_node[0], next_node[1]):
                    new_cost = cost_so_far[current] + (math.sqrt(2) if dx != 0 and dy != 0 else 1)
                    if next_node not in cost_so_far or new_cost < cost_so_far[next_node]:
                        cost_so_far[next_node] = new_cost
                        priority = new_cost + self.euclidean_distance(end, next_node)
                        heapq.heappush(frontier, (priority, next_node))
                        came_from[next_node] = current


        return came_from

    def reconstruct_path(self, came_from, start, end):
        current = end
        path = []
        while current != start:
            path.append(current)
            current = came_from[current]
        path.append(start)
        path.reverse()
        return path

    
if __name__ == "__main__":
    
    grid = Grid(2000, 3000, 50, (0, 0), 0, (1500, 1500))
    astar = Astar()

    for _ in range(500):
        x, y = random.randint(0, grid.width - 1), random.randint(0, grid.height - 1)
        for dx in range(-1, 2):
            for dy in range(-1, 2):
                grid.add_point(x + dx, y + dy, 'avoid')


    cell_grid = grid.convert_to_cell_grid_object()

    came_from = astar.a_star_search_diagonal(cell_grid, cell_grid.start, cell_grid.end)
    path = astar.reconstruct_path(came_from, cell_grid.start, cell_grid.end)
    print(path)

    cell_grid.path = path

    waypoints = grid.bulk_convert_to_normal_coordinates(cell_grid.get_waypoints(2))
    if waypoints[-1] != grid.convert_to_normal_coordinates(cell_grid.end):
        waypoints.append(grid.convert_to_normal_coordinates(cell_grid.end))
    if waypoints[0] != grid.convert_to_normal_coordinates(cell_grid.start):
        waypoints.insert(0, grid.convert_to_normal_coordinates(cell_grid.start))


    print(waypoints)
    obstacles = grid.bulk_convert_to_normal_coordinates(cell_grid.get_obstacles())

    robot_path = Spline(waypoints)
    robot_path.plot_path(obstacles)