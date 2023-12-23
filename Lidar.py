import time
from gpiozero import AngularServo, Device
from gpiozero.pins.pigpio import PiGPIOFactory
import numpy as np
import matplotlib.pyplot as plt
import board
from adafruit_vl53l0x import VL53L0X
from busio import I2C
# import Grid


class Lidar:

    def __init__(self, servo_pin=13, half_angle_sweep=45):
        Device.pin_factory = PiGPIOFactory()
        self.servo = AngularServo(servo_pin, min_angle=-half_angle_sweep, max_angle=half_angle_sweep, pin_factory=Device.pin_factory)
        self.half_angle_sweep = half_angle_sweep
        self.direction = 1

        i2c = I2C(board.SCL, board.SDA)
        self.tof = VL53L0X(i2c)
        self.tof.measurement_timing_budget = 17090
    
    def calculate_angles(self, start_angle, end_angle, num_measurements):
        angles = np.linspace(np.radians(start_angle), np.radians(end_angle), num_measurements)
        return angles
    
    def plot_distances(self, distances, angles):
        fig = plt.figure()
        ax = fig.add_subplot(111, polar=True)
        ax.set_ylim(0, distances.max())
        ax.set_yticks(np.arange(0, distances.max(), 100))
 
        ax.set_thetamin(-60)
        ax.set_thetamax(60)

        ax.set_theta_zero_location("N")
        ax.set_theta_direction(-1)

        ax.scatter(angles, distances, color='r', marker='.') 
        plt.title('Distance measurements in 120 degree area')
        plt.savefig("output.jpg")

    def perform_sweep(self, servo, start_angle, end_angle, num_measurements, interval_time, direction=1):

        distances = np.zeros(num_measurements)

        servo.angle = start_angle * direction

        for i in range(num_measurements):
            time.sleep(interval_time)
            distance = self.measure_distance()
            if distance < 2000:
                distances[i*direction] = distance
            else:
                distances[i*direction] = 2000

        return distances
    
    def measure_distance(self):
        distance = self.tof.range
        return distance

    def perform_sweep_new(self, servo, half_angle_sweep, direction=1):
        
        servo_speed = 540

        angle_distance_pairs = []

        start_angle = half_angle_sweep * direction
        end_angle = half_angle_sweep * direction * -1

        servo.angle = end_angle

        start_time = time.time()
        while True:
            
            distance = self.measure_distance()

            elapsed_time = time.time() - start_time
            current_angle = start_angle + (servo_speed * elapsed_time * direction * -1)

            angle_distance_pairs.append((current_angle, min(distance, 2000)))

            if direction == 1 and current_angle <= end_angle:
                break
            elif direction == -1 and current_angle >= end_angle:
                break

        return angle_distance_pairs

    
    def push_distance_angle(self, distances, angles):
        distance_angle = []
        for i in range(len(distances)):
            distance_angle.append([distances[i], angles[i]])
        return distance_angle
    
    def print_array(self, array):
        """Prints an array of 0 and 1. If the array value is greater than 500 it prints a 0 otherwise it prints a 1"""
        for i in range(len(array)):
            if array[i] > 500:
                print("0", end="")
            else:
                print("1", end="")
        print()


    def start_sweeping(self, shared_grid = None):
        sweep_count = 0
        self.tof.start_continuous()
        try:
            while True:
                distances_angles = self.perform_sweep_new(self.servo, self.half_angle_sweep, self.direction)
                if shared_grid is not None:
                    points = shared_grid.angle_distance_to_point(distances_angles)
                    shared_grid.bulk_add_points(points, 'avoid')
                    if sweep_count < 4:
                        shared_grid.update_grid(points)
                        sweep_count = 0
                self.direction = -self.direction
                sweep_count += 1
                time.sleep(0.001)
        except KeyboardInterrupt:
            self.tof.stop_continuous()
        finally:
            self.tof.stop_continuous()


if __name__ == "__main__":
    lidar = Lidar()
    lidar.start_sweeping()