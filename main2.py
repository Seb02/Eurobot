from Grid import Astar, Grid, CellGrid
# from Lidar import Lidar
from main import Robot, Spline
import time
import threading
import socket
import pickle
import time
import os
import matplotlib.pyplot as plt
import datetime
import os
import math


def plot_and_save_grid(grid, filename_prefix="grid_plot_"):
    fig, ax = plt.subplots()

    width = grid.width 
    height = grid.height 
    obstacles = grid.obstacles 
    waypoints = grid.path 
    spline_path = grid.get_current_spline()
    robot_pos = grid.robot_pos 
    robot_angle = grid.robot_angle 

    ax.set_xlim(0, width)
    ax.set_ylim(0, height)

    for obstacle in obstacles:
        ax.scatter(*obstacle, marker='s', color='black', label='Obstacle' if obstacle == obstacles[0] else "")

    for waypoint in waypoints:
        ax.scatter(*waypoint, marker='o', color='blue', label='Waypoint' if waypoint == waypoints[0] else "")

    if robot_pos:
        robot_marker = ax.scatter(*robot_pos, marker='o', color='green', label='Robot')
        orientation_vector = (math.cos(math.radians(robot_angle)), math.sin(math.radians(robot_angle)))

        ax.arrow(*robot_pos, *(x * 10 for x in orientation_vector), color='green', label='Orientation')

    if spline_path:
        print(spline_path[0])
        ax.plot(spline_path[0], spline_path[1], 'b-', label='spline')

    ax.set_title('Grid with Obstacles and Waypoints')
    ax.set_xlabel('X Coordinate')
    ax.set_ylabel('Y Coordinate')
    ax.legend()
    
    if not os.path.exists('figures'):
        os.makedirs('figures')

    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    plt.savefig(f"figures/{filename_prefix}{timestamp}.png")
    plt.close() 

def plotting_thread_function(shared_grid, plot_interval=2):
    while True:
        plot_and_save_grid(shared_grid)
        time.sleep(plot_interval)


class ThreadSafeGrid(Grid):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.lock = threading.Lock()
        self.path = []

    def update_grid(self, points):
        with self.lock:
            self.refresh_grid(points)

    def get_data_for_astar(self):
        with self.lock:
            return self.convert_to_cell_grid_object()

    def update_path(self, new_path):
        with self.lock:
            self.path = new_path
    
    def get_current_spline(self):
        """returns the spline object from the waypoints"""
        with self.lock:
            return Spline(self.path).get_spline()


    @staticmethod
    def from_serializable_data(data):
        grid = ThreadSafeGrid(data['grid_width'], data['grid_height']) 
        grid.path = data['path']
        grid.obstacles = set(data['obstacles'])
        grid.robot_pos = data['robot_pos']
        grid.robot_angle = data['robot_angle']
        grid.goal = data['goal']

        return grid

def astar_thread_function(shared_grid, update_interval):
    astar = Astar()
    while True:
        time.sleep(update_interval)
        cell_grid = shared_grid.get_data_for_astar()
        start, end = cell_grid.start, cell_grid.end

        came_from = astar.a_star_search_diagonal(cell_grid, start, end)
        path = astar.reconstruct_path(came_from, start, end)

        cell_grid.path = path
        
        waypoints = shared_grid.bulk_convert_to_normal_coordinates(cell_grid.get_waypoints(2))
        if len(waypoints) > 0:
            if waypoints[-1] != shared_grid.convert_to_normal_coordinates(cell_grid.end):
                waypoints.append(shared_grid.convert_to_normal_coordinates(cell_grid.end))
            if waypoints[0] != shared_grid.convert_to_normal_coordinates(cell_grid.start):
                waypoints.insert(0, shared_grid.convert_to_normal_coordinates(cell_grid.start))

        shared_grid.update_path(waypoints)


if __name__ == "__main__":
    initial_position = (0, 50)
    initial_orientation = 0
    waypoints = [(0, 50), (10, 50), (20, 50), (30, 50), (90, 50)]

    shared_grid = ThreadSafeGrid(100, 100, 10, initial_position, 0, (90, 0))

    plotting_thread = threading.Thread(target=plotting_thread_function, args=(shared_grid,))
    plotting_thread.start()

    lidar = Lidar()
    lidar_thread = threading.Thread(target=lidar.start_sweeping, args=(shared_grid,))
    lidar_thread.start()

    robot = Robot(waypoints)
    robot.shared_grid = shared_grid 
    robot.controller.shared_grid = shared_grid 
    robot_thread = threading.Thread(target=robot.run, args=(initial_position,))
    robot_thread.start()

    astar_thread = threading.Thread(target=astar_thread_function, args=(shared_grid, 0.1))
    astar_thread.start()