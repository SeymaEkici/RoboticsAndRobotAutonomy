import matplotlib.pyplot as plt
import the1_sim
import numpy as np
import csv

class Robot:
    class Position:  # Create states of robot
        def __init__(self, x, y, theta):
            self.x = x
            self.y = y
            self.theta = theta

    class Parameters:  # Physical parameters of robot
        def __init__(self, cg, length, width, wheel_length, wheel_width, dt):
            self.cg = cg  # Location of center of gravity w.r.t. rear wheel
            self.length = length  # Wheelbase length
            self.width = width  # Width of the robot
            self.wheel_length = wheel_length  # Wheel length for plot purposes
            self.wheel_width = wheel_width  # Wheel width for plot
            self.dt = dt

    class Input:  # Define the input of robot as velocity and steering angle
        def __init__(self, vel, steer):
            self.vel = vel
            self.steer = steer

    def __init__(self, position, parameters, input_):
        self.position = position
        self.parameters = parameters
        self.input = input_


# Function to update the position of robot according to bicycle kinematic model
def bicycle_kinematic(robot):
    theta = np.radians(robot.position.theta)
    steer = np.radians(robot.input.steer)

    beta = np.arctan((robot.parameters.cg / robot.parameters.length) * np.tan(steer))

    robot.position.x += robot.input.vel * np.cos(theta + beta) * robot.parameters.dt
    robot.position.y += robot.input.vel * np.sin(theta + beta) * robot.parameters.dt
    robot.position.theta += np.degrees((robot.input.vel / robot.parameters.length) * np.sin(beta) * robot.parameters.dt)

    return robot.position


# Function to read input data from CSV file
def robot_input(robot, i):
    with open('applied_input.csv', mode='r') as file:
        reader = csv.reader(file)
        # Store all rows in a list, skipping the header
        input_data = [row[0].split(';') for row in reader][1:]  # Split the string into columns
        input_length = len(input_data)
        if i < input_length:
            robot.input.vel = float(input_data[i][1])
            robot.input.steer = float(input_data[i][2])


# Main function
if __name__ == "__main__":
    fig, ax = plt.subplots(figsize=(12, 12))
    simTime = 0.1

    # Initialize robot parameters and state
    position = Robot.Position(x=0, y=0, theta=0)
    input_ = Robot.Input(vel=0, steer=0)
    params = Robot.Parameters(cg=8, length=16, width=8, wheel_length=5, wheel_width=1, dt=simTime)

    robot = Robot(position=position, input_=input_, parameters=params)
    
    # Run the simulation
    the1_sim.simulation(fig, ax, robot)
