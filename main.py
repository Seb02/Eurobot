import numpy as np
from scipy.interpolate import BSpline, make_interp_spline
import matplotlib.pyplot as plt
import serial
import time
from enum import Enum
import threading

class State(Enum):
    Idle = 0
    Running = 1
    Error = 2
    Debug = 3

class Type(Enum):
    Null = 0
    Debug = 1
    Error = 2
    Angular = 3
    Linear = 4
    Position = 5
    Orientation = 6

class Spline:
    def __init__(self, waypoints, k=3):
        self.waypoints = waypoints
        self.k = k
        self.c, self.x, self.y = None,None,None
        self.total_length, self.t = 0,0

        self._create_bspline()
    
    def _create_bspline(self):
        waypoints_x, waypoints_y = zip(*self.waypoints)
        n = len(waypoints_x)

        t = [0]
        for i in range(1, n):
            t.append(t[-1] + np.sqrt((waypoints_x[i] - waypoints_x[i-1])**2 + (waypoints_y[i] - waypoints_y[i-1])**2))
            self.total_length = t[i]*1

        t = np.array(t) / t[-1]  

        spline_x = make_interp_spline(t, waypoints_x, k=self.k)
        spline_y = make_interp_spline(t, waypoints_y, k=self.k)
        
        debug_t_values = np.linspace(0, 1, n)
        debug_y_values = spline_y(debug_t_values)

        self.t = t
        self.c = np.c_[waypoints_x, waypoints_y]
        self.x = spline_x
        self.y = spline_y

    def get_nearest_point(self, current_position, current_t, search_range=.02, search_points=10):
        min_distance = float('inf')
        nearest_point = (0,0)
        
        offset = search_range / search_points
        while offset < search_range:
            t = current_t + offset
            if t < 0:
                t = 0
            if t > 1:
                if nearest_point == (0,0):
                    nearest_point =  current_position
                return nearest_point


            point = self.x(t), self.y(t)
            distance = np.linalg.norm(np.array(current_position) - np.array(point))
            
            if distance < min_distance:
                print("distance: ", distance)
                min_distance = distance
                nearest_point = point

            offset += search_range / search_points
            
        
        return nearest_point
    
    def get_curvature(self, t):
        dx_dt = self.x(t, 1)
        dy_dt = self.y(t, 1)

        d2x_dt2 = self.x(t, 2)
        d2y_dt2 = self.y(t, 2)
        return (dx_dt * d2y_dt2 - dy_dt * d2x_dt2) / (dx_dt**2 + dy_dt**2)**(3/2)
    
    def get_spline(self):
        t = np.linspace(0, 1, 100)
        x = self.x(t)
        y = self.y(t)
        return x, y
    
    def plot_path(self, obstacles):
        t = np.linspace(0, 1, 100)
        x = self.x(t)
        y = self.y(t)

        plt.plot(x, y, 'b-', label='spline')
        plt.plot(self.c[:,0], self.c[:,1], 'ro', label='control points')
        #plot obstacles from a list of tuples containing x,y coordinates
        for obstacle in obstacles:
            plt.plot(obstacle[0], obstacle[1], 'ro', label='obstacle')
        plt.legend(loc='best')
        plt.axis([min(x)-1, max(x)+1, min(y)-1, max(y)+1])
        #plot to png
        plt.savefig('path.png')
    
class PIDController:
    def __init__(self, kp = 0.02, ki = 0.0, kd = 0.0, integral_limit=100):  

        self.Kp = kp
        self.Ki = ki
        self.Kd = kd

        self.integral_limit = 100

        self.last_error = 0
        self.error = 0
        self.error_sum = 0
        self.error_diff = 0

        self.output_linear_velocity = 0
        self.output_angular_velocity = 0

    def calculate_positional_error(self, position, orientation, nearest_point, dt):
        self.dt = dt
        vector_to_nearest = np.array(nearest_point) - np.array(position)
        orientation_vector = np.array([np.cos(orientation), np.sin(orientation)])
        cross_product_z = np.cross(np.append(orientation_vector, 0), np.append(vector_to_nearest, 0))[2]
        error_magnitude = np.linalg.norm(vector_to_nearest)
        signed_error = -(np.sign(cross_product_z) * error_magnitude)

        self.last_error = self.error
        self.error_sum += signed_error
        self.error_diff = (signed_error - self.last_error) / self.dt
        self.error = signed_error


    def adjust_velocities(self, target_linear_velocity, target_angular_velocity):
    
        if self.error_sum > self.integral_limit:
            self.error_sum = self.integral_limit
        elif self.error_sum < -self.integral_limit:
            self.error_sum = -self.integral_limit

        corrected_angular_velocity = target_angular_velocity + self.Kp * self.error + self.Ki * self.error_sum + self.Kd * self.error_diff
        self.output_linear_velocity =  target_linear_velocity
        self.output_angular_velocity = corrected_angular_velocity

class RobotController:

    def __init__(self, spline, PID, max_speed, min_speed, max_acceleration=50, max_deceleration=150):

        self.max_speed = max_speed
        self.min_speed = min_speed
        self.max_acceleration = max_acceleration
        self.max_deceleration = max_deceleration

        self.spline = spline

        self.current_s = 0
        self.position = (0, 0)
        self.orientation = 0

        self.dt = 0.1
        
        self.PID = PID

    def calculate_braking_distance(self, current_speed):
        return (current_speed ** 2) / (2 * self.max_deceleration)
    
    def update_position_on_grid(self, shared_grid):

        shared_grid.move_robot(self.position[0]*10, self.position[1]*10, self.orientation)
    
    def update_position(self, linear_velocity, angular_velocity):
        self.orientation += angular_velocity * self.dt
        dx = linear_velocity * self.dt * np.cos(self.orientation)
        dy = linear_velocity * self.dt * np.sin(self.orientation)
        self.position = (self.position[0] + dx, self.position[1] + dy)

    def get_target_velocities(self, current_linear_velocity, current_angular_velocity):
        self.current_s += current_linear_velocity * self.dt
        t_normalized = self.current_s / self.spline.total_length
        braking_distance = self.calculate_braking_distance(current_linear_velocity)
        remaining_distance = self.spline.total_length - self.current_s
        
        if braking_distance >= remaining_distance:
            print("decelerating")
            target_linear_velocity = max(current_linear_velocity - self.max_deceleration * self.dt, 0)
        else:
            print("accelerating")
            target_linear_velocity = min(current_linear_velocity + self.max_acceleration * self.dt, self.max_speed)

        target_linear_velocity = max(target_linear_velocity, self.min_speed)

        curvature = self.spline.get_curvature(t_normalized)
        target_angular_velocity = curvature * target_linear_velocity
        nearest_point = self.spline.get_nearest_point(self.position, t_normalized)
       
        self.PID.calculate_positional_error(self.position, self.orientation, nearest_point, self.dt)
        self.PID.adjust_velocities( target_linear_velocity, target_angular_velocity)

        
        if self.current_s / self.spline.total_length >= 1:
            return 0, 0

        return self.PID.output_linear_velocity, self.PID.output_angular_velocity
    
    def get_output_Velocities(self, current_linear_velocity, current_angular_velocity, dt):
        self.dt = dt
        BS = 5.5
        target_linear_velocity, target_angular_velocity = self.get_target_velocities(current_linear_velocity, current_angular_velocity)
        output_right_velocity =max(min((target_linear_velocity + target_angular_velocity * BS)*1.4,100),-100)
        output_left_velocity = max(min((target_linear_velocity - target_angular_velocity * BS)*1.4,100),-100)
        return output_left_velocity, output_right_velocity

class Message:
    def __init__(self, message):
        self.message = message
        self.type = Type.Null
        self.value = ""
        self.unpack()
    
    def getDataFromString(self):
        string = self.message.split(":")
        mtype = Type.Null
        if string[0] == "linear":
            mtype = Type.Linear
            self.type,self.value = mtype, float(string[1])
        elif string[0] == "angular":
            mtype = Type.Angular
            self.type,self.value = mtype, float(string[1])
        elif string[0] == "XY":
            mtype = Type.Position
            self.type,self.value = mtype, self.getTupleFromString(string[1])
        elif string[0] == "angle":
            mtype = Type.Orientation
            self.type,self.value = mtype, float(string[1])
        
    def getTupleFromString(self, string):
        string = string.split("(")
        string = string[1].split(")")
        string = string[0].split(",")
        return (float(string[0]), float(string[1]))


    def unpack(self):
        if self.message[0] == "_":
            self.type = Type.Debug
            self.value = self.message[1:]

        elif self.message[0] == "!":
            self.type = Type.Error
            self.value = self.message[1:]

        else:
            self.getDataFromString()

class CommunicationInterface:
    def __init__(self, port='/dev/ttyAMA0', baudrate=115200, timeout=1, debug = True):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.serial = None
        self.debugMessage = debug
    
    def openSerial(self):
        self.serial = serial.Serial(self.port, baudrate=self.baudrate, timeout=self.timeout)
        self.serial.flushInput()
        self.serial.flushOutput()
    
    def closeSerial(self):
        self.serial.close()

    def sendVelocities(self, linear_velocity, angular_velocity):
        message = f"{linear_velocity}:{angular_velocity}$\n"
        self.serial.write(message.encode('utf-8'))

    def sendInit(self, position=(0,0)):
        message = "RESET\n"
        message += f"setPosition {position}\n"
        self.serial.write(message.encode('utf-8'))

    def messageInbound(self):
        return self.serial.inWaiting() > 0

    def receive(self):
        return Message(self.serial.readline().decode('utf-8').rstrip())

    def debug(self, message):
        if self.debugMessage:
            print(f"Debug: {message}")
    def status(self, message):
        if self.debugMessage:
            print(f"Status: {message}")
    def com(self, message):
        if self.debugMessage:
            print(f"Message: {message}")

class Robot:
    def __init__(self, waypoints, timeout=10, max_speed=100, min_speed=20):
        self.spline = Spline(waypoints)
        self.PID = PIDController()
        self.controller = RobotController(self.spline, self.PID, max_speed, min_speed)
        self.communication = CommunicationInterface()
        self.state = State.Idle
        self.timeout = timeout
        self.shared_grid = None

        #self.setup_input_listener()

    def setup_input_listener(self):
        input_thread = threading.Thread(target=self.input_listener, daemon=True)
        input_thread.start()

    def input_listener(self):
        while True:
            user_input = input()
            if user_input.lower() == "stop":
                self.state = State.Idle
            

    def run(self, position=(0,0)):
        linear_velocity = 1
        angular_velocity = 0
        linear_flag = False
        angular_flag = False
        start_time = 0
        last_time = time.time()

        if self.shared_grid:
            self.controller.update_position_on_grid(self.shared_grid)

        while self.state == State.Idle:
            command = input("Running or Debug?\nEnter a command: ")
            if command == "":
                self.communication.com("Please enter a command")
            elif command[0].capitalize() == "R":
                self.state = State.Running
                self.communication.com("Robot is now running")
            elif command[0].capitalize() == "D":
                self.state = State.Debug
                self.communication.com("Robot is now in debug mode")

        if self.state == State.Running:
            self.communication.openSerial()
            self.communication.sendInit(position)

        elif self.communication.serial is not None:
            self.communication.closeSerial()

        while self.state == State.Running:
            if start_time == 0:
                start_time = time.time()

            if self.shared_grid:
                self.controller.update_position_on_grid(self.shared_grid)

            if time.time() - start_time > self.timeout:
                self.communication.debug("timeout")
                self.communication.sendVelocities(0, 0)
                self.state = State.Idle
                break

            if self.communication.messageInbound():
                message = self.communication.receive()
                if message.type == Type.Debug:
                    self.communication.comm(message.value)
                    pass
                elif message.type == Type.Error:
                    self.communication.comm(message.value)
                    self.state = State.Error
                elif message.type == Type.Linear:
                    linear_velocity = message.value
                    self.communication.com(f"linear: {linear_velocity}")
                    linear_flag = True
                elif message.type == Type.Angular:
                    angular_velocity = message.value
                    self.communication.com(f"angular: {angular_velocity}")
                    angular_flag = True
                elif message.type == Type.Position:
                    self.controller.position = message.value
                    self.communication.com(f"position: {self.controller.position}")
                elif message.type == Type.Orientation:
                    self.controller.orientation = message.value
                    self.communication.com(f"orientation: {self.controller.orientation}")
                

            if linear_flag and angular_flag:
                

                current_time = time.time()
                dt = current_time - last_time
                last_time = current_time

                linear_flag = False
                angular_flag = False

                left_velocity, right_velocity = self.controller.get_output_Velocities(linear_velocity, angular_velocity, dt)
                self.communication.sendVelocities(left_velocity, right_velocity)
                self.communication.debug((left_velocity, right_velocity))

                if left_velocity == 0 and right_velocity == 0:

                    self.state = State.Idle
                    self.communication.status("Robot is now idle")
                    break

        while self.state == State.Debug:
            dt = 0.1
            time.sleep(dt)
            if linear_velocity == 0 and angular_velocity == 0:

                self.state = State.Idle
                self.communication.status("Robot is now idle")
                break

            linear_velocity, angular_velocity = self.controller.get_target_velocities(linear_velocity, angular_velocity)
            left_velocity, right_velocity = self.controller.get_output_Velocities(linear_velocity, angular_velocity, dt)
            self.communication.debug((left_velocity, right_velocity))

if __name__ == "__main__":
    # waypoints = [(0, 0), (10, 10), (20, 0), (30, 10), (40, 0)]
    waypoints = [(0, 0), (10, 0), (20, 0), (30, 30)]
    robot = Robot(waypoints)
    while True:
        robot.run()