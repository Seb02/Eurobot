from main2 import ThreadSafeGrid
from main2 import plot_and_save_grid

class TestPlotAndSaveGrid:
    def test_plot_and_save_grid(self):
        grid = ThreadSafeGrid(30, 30)
        grid.path = [(0, 0), (5, 5), (10, 10), (2, 20)]
        grid.obstacles = [(1, 2), (3, 4)]
        grid.robot_pos = (13, 14)
        grid.robot_angle = 15
        spline = grid.get_current_spline()
        print(spline)
        plot_and_save_grid(grid, "test_")

if __name__ == "__main__":
    test = TestPlotAndSaveGrid()
    test.test_plot_and_save_grid()

