import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import numpy as np
from math import pi


if __name__ == "__main__":
    fig, ax = plt.subplots(figsize=(8, 8))
    min=-10
    max=60
    ax.set_xlim(min, max)
    ax.set_ylim(min, max)
    # Add gridlines
    ax.set_xticks(np.arange(min, max, 10))
    ax.set_xticks(np.arange(min, max, 1), minor=True)
    ax.set_yticks(np.arange(min, max, 10))
    ax.set_yticks(np.arange(min, max, 1), minor=True)
    ax.grid(which='minor', alpha=0.2)
    ax.grid(which='major', alpha=0.5)
    ax.set_aspect('equal', adjustable='box')

    # a rectangle which has length 10, with 6 and bottom left corner is located at (0,1) and turned 45 degrees
    rec1 = Rectangle((0, 1), 10, 6, angle=np.degrees(pi/4), fill=True, alpha=0.5, edgecolor='black', linewidth=2)
    ax.add_patch(rec1)

    # a rectangle L=10, W=6, center of rectangle located at (20,25) with 0 turning
    L= 10
    W= 6
    x_bottomleft = 20 - L/2
    y_bottomleft = 25 - W/2
    rec2 = Rectangle((x_bottomleft, y_bottomleft), L, W, angle=0, fill=True, alpha=0.5, edgecolor='black', linewidth=2)
    ax.add_patch(rec2)
    ax.plot(20,25,'ro')
    ax.plot(x_bottomleft,y_bottomleft,'go')

    # a rectangle L=10, W=6, center of rectangle located at (45,10) and turned 30 degrees
    # You know the position of the center at the world coordinate frame
    # bl means bottom-left, R means Rectangle coordinate Frame, W means world coordinate frame
    x_R_W = 45
    y_R_W = 10
    # Imagine a coordinate frame located at the center of rectangle
    # The bottom-left position of the rectangle is always constant on that coordinate frame
    x_bl_R = -L/2
    y_bl_R = -W/2

    # Now, how we will find the the positions of bottom-left of the rectangle in the world coordinate frame?
    # X_bl_W = ?, Y_bl_W = ?
    x_bl_W = x_R_W + x_bl_R * np.cos(pi/6) - y_bl_R * np.sin(pi/6)
    y_bl_W = y_R_W + x_bl_R * np.sin(pi/6) + y_bl_R * np.cos(pi/6)

    rec3 = Rectangle((x_bl_W, y_bl_W), L, W, angle=np.degrees(pi/6), fill=True, alpha=0.5, edgecolor='black', linewidth=2)
    ax.add_patch(rec3)
    ax.plot(x_R_W,y_R_W,'ro')
    ax.plot(x_bl_W,y_bl_W,'go')

    plt.show()