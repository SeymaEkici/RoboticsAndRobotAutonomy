Robot Path Planning Simulation
==============================

Overview
--------

This project simulates a mobile robot navigating an environment using different path planning algorithms (Bug0, Bug1, and Bug2). The robot must reach a target position while avoiding obstacles using a combination of movement strategies and bump sensors.

Project Structure
-----------------

```
THE2/
├── the2.py             # Main script for simulation and control
├── bug_planners.py     # Implementation of Bug0, Bug1, and Bug2 algorithms
├── the2_simulation.py  # Visualization and environment setup

```

Features
--------

-   **Robot Movement:** Implements a differential drive kinematic model.
-   **Bump Sensors:** Detects obstacles and updates robot status.
-   **Path Planning Algorithms:**
    -   **Bug0:** Moves to the goal and follows obstacles until it finds a free path.
    -   **Bug1:** Records the shortest distance along the obstacle boundary before resuming motion toward the goal.
    -   **Bug2:** Stores a straight-line equation to help navigate around obstacles more efficiently.
-   **Visualization:** Uses `matplotlib` to display the environment, obstacles, and robot movement in real-time.

Dependencies
------------

Ensure you have the following Python libraries installed:

```
pip install numpy matplotlib shapely

```

How to Run
----------

1.  Navigate to the project directory:

    ```
    cd THE2

    ```

2.  Run the simulation:

    ```
    python the2.py

    ```

Configuration
-------------

-   Modify `robot.planner` in `the2.py` to change the path planning algorithm:

    ```
    robot = Robot(position=position, prev_position=prev_position, input=input_, memory=memory, planner=0)

    ```

    -   `planner=0` → Bug0
    -   `planner=1` → Bug1
    -   `planner=2` → Bug2

License
-------

This project is for educational purposes. Feel free to modify and extend it.
