Robot Kinematic Simulation
==========================

Overview
--------

This project simulates the motion of a robot using a bicycle kinematic model. It takes velocity and steering angle inputs from a CSV file and visualizes the robot's movement on a 2D plane.

Project Structure
-----------------

```
THE1/
│-- the1_main.py      # Main script to initialize and run the simulation
│-- the1_sim.py       # Handles animation and visualization
│-- the_hint.py       # Provides example transformations and coordinate frame calculations
│-- applied_input.csv # Input file with velocity and steering angle data
│-- README.md         # Project documentation

```

Requirements
------------

Ensure you have the following Python libraries installed:

-   `matplotlib`
-   `numpy`

You can install missing dependencies using:

```
pip install matplotlib numpy

```

How to Run
----------

1.  Prepare an input file `applied_input.csv` with velocity and steering angle data.
2.  Run the main script:

    ```
    python THE1/the1_main.py

    ```

3.  The simulation will animate the robot's movement based on the given inputs.

File Descriptions
-----------------

### `the1_main.py`

-   Defines the `Robot` class with nested classes for `Position`, `Parameters`, and `Input`.
-   Implements the `bicycle_kinematic` function to update the robot's position.
-   Reads input commands from `applied_input.csv`.
-   Calls `the1_sim.simulation()` to start the animation.

### `the1_sim.py`

-   Handles visualization and animation using `matplotlib`.
-   `plot_robot()`: Draws the robot on the plot.
-   `plot_wheel()`: Plots the robot's front wheel with steering angle.
-   `simulation()`: Sets up and runs the animation loop.

### `the_hint.py`

-   Demonstrates coordinate frame transformations with example rectangles.

Input Format (`applied_input.csv`)
----------------------------------

The CSV file should have the following format:

```
Time;Velocity;Steering Angle
0;1.0;10
1;1.2;15
... (more data rows)

```

-   Time: Simulation step (not used directly in calculations)
-   Velocity: Robot's forward speed (units per time step)
-   Steering Angle: Angle of the front wheel (degrees)

Simulation Explanation
----------------------

The robot follows a bicycle kinematic model:

-   The center of gravity (CG) influences turning behavior.
-   The steering angle affects the robot's orientation.
-   The movement is updated in discrete time steps (`dt`).

Customization
-------------

-   Modify `the1_main.py` to change robot parameters like length, width, or CG position.
-   Adjust the `applied_input.csv` file to test different motion scenarios.
-   Edit `the1_sim.py` to change animation properties such as axis limits or grid settings.

License
-------

This project is for educational purposes. Feel free to modify and extend it.
