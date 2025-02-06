import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from matplotlib.animation import FuncAnimation
import numpy as np
import csv
import the1_main

# Function to plot the robot body
def plot_robot(ax, robot, previous_robot=None, previous_wheel=None):
    
    x_BL_R = -robot.parameters.length / 2
    y_BL_R = -robot.parameters.width / 2

    if previous_robot is not None:
        previous_robot.remove()
    if previous_wheel is not None:
        previous_wheel.remove()

    
    theta = np.radians(robot.position.theta)
    x_BL_W = robot.position.x + (x_BL_R * np.cos(theta) - y_BL_R * np.sin(theta))
    y_BL_W = robot.position.y + (x_BL_R * np.sin(theta) + y_BL_R * np.cos(theta))

    robot_pos = Rectangle((x_BL_W, y_BL_W), robot.parameters.length, robot.parameters.width,
                          angle=np.degrees(theta), fill=True, alpha=0.5, edgecolor='black', linewidth=2)
    ax.add_patch(robot_pos)

    wheel_pos = plot_wheel(ax, robot)
    
    return robot_pos, wheel_pos


# Function to plot the green filled rectangle with a midpoint on the right side of the red rectangle
def plot_wheel(ax, robot):
    x_wheel_wh = -robot.parameters.wheel_length / 2
    y_wheel_wh = -robot.parameters.wheel_width / 2

    steer = np.radians(robot.input.steer)
    
    
    x_wheel_R = robot.parameters.length / 2
    y_wheel_R = 0
    
    
    theta = np.radians(robot.position.theta)
    x_wheel_W = robot.position.x + (x_wheel_R * np.cos(theta) - y_wheel_R * np.sin(theta))
    y_wheel_W = robot.position.y + (x_wheel_R * np.sin(theta) + y_wheel_R * np.cos(theta))

    
    wheel_pos = Rectangle((x_wheel_W, y_wheel_W), robot.parameters.wheel_length, robot.parameters.wheel_width,
                          angle=np.degrees(theta + steer), facecolor='green', edgecolor='none')
    ax.add_patch(wheel_pos)

    return wheel_pos


def simulation(fig, ax, robot):
    # Set up the empty plot with specified boundaries and aspect ratio
    ax.set_xlim(-20, 100)  # Set x-axis boundaries
    ax.set_ylim(-20, 100)  # Set y-axis boundaries
    ax.set_aspect('equal', adjustable='box')  # Preserve aspect ratio

    # Add gridlines
    ax.set_xticks(np.arange(-20, 100, 10))
    ax.set_xticks(np.arange(-20, 100, 2), minor=True)
    ax.set_yticks(np.arange(-20, 100, 10))
    ax.set_yticks(np.arange(-20, 100, 2), minor=True)

    ax.grid(which='minor', alpha=0.2)
    ax.grid(which='major', alpha=0.5)


    # Add labels
    ax.set_xlabel('X-axis')
    ax.set_ylabel('Y-axis')

    # Initialize the parameters
    previous_robot_pos = None
    previous_wheel_pos = None

    # Initialize the previous position to plot traveled path
    prev_x, prev_y = robot.position.x, robot.position.y

    # Initialization function for the animation
    def init():
        return []

    # Animation function to update the position of the red rectangle's center and rotation angle
    def animate(frame):
        nonlocal previous_robot_pos, previous_wheel_pos, prev_x, prev_y, robot  # Use the previous_robot and prev_x, prev_y from the outer scope
        
        #Take the input that will applied
        the1_main.robot_input(robot,frame)

        # Update the position with bicycle kinematic model
        robot.position = the1_main.bicycle_kinematic(robot)

        # Call the plot_robot function to update the robot position
        previous_robot_pos, previous_wheel_pos = plot_robot(ax, robot, previous_robot_pos, previous_wheel_pos)

        # Plot the traveled path
        line = ax.plot([prev_x, robot.position.x], [prev_y, robot.position.y], 'r.', lw=1, ls='dotted')
        prev_x, prev_y = robot.position.x, robot.position.y

    # Create the animation
    ani = FuncAnimation(fig, animate, frames=600, init_func=init, blit=False, repeat=False, interval=20)
    # Increase interval (200 seems good) to slow down the sim and take easy screenshots
    
    # Show the animation
    plt.show()