Robot Simulation and Path Planning Projects
===============================================================

This repository contains Python code for three related projects focused on simulating robot movement and path planning. These projects model different aspects of robot kinematics, pathfinding algorithms, and grid-based environments. They include:

1.  **Robot Kinematic Simulation**: Simulates the motion of a robot using a bicycle kinematic model.
2.  **Robot Path Planning Simulation**: Implements path planning algorithms (Bug0, Bug1, and Bug2) to navigate a robot around obstacles.
3.  **Search Algorithms on a Grid**: Visualizes search algorithms (BFS, DFS, Dijkstra, A*) on a grid-based environment to find the shortest path from start to goal while avoiding obstacles.

* * * * *

Project Structure
-----------------

The repository is divided into three main folders, each containing the relevant files for the respective project.

### Project 1: Robot Kinematic Simulation (THE1)

```
THE1/
│-- the1_main.py      # Main script to initialize and run the simulation
│-- the1_sim.py       # Handles animation and visualization
│-- the_hint.py       # Provides example transformations and coordinate frame calculations
│-- applied_input.csv # Input file with velocity and steering angle data
│-- README.md         # Project documentation

```

### Project 2: Robot Path Planning Simulation (THE2)

```
THE2/
├── the2.py             # Main script for simulation and control
├── bug_planners.py     # Implementation of Bug0, Bug1, and Bug2 algorithms
├── the2_simulation.py  # Visualization and environment setup

```

### Project 3: Search Algorithms on a Grid (THE3)

```
THE3/
├── THE3.py            # Main script for grid-based world and search algorithms
├── Node.py            # Defines grid nodes (obstacle or free space)
├── GridWorld.py       # Sets up grid with start, goal, and obstacles
├── SearchAlgorithms.py # Implements BFS, DFS, Dijkstra, and A*

```

* * * * *

Requirements
------------

Ensure the following Python libraries are installed for all three projects:

-   `matplotlib`
-   `numpy`
-   `shapely` (for path planning simulation)

Install dependencies using pip:

```
pip install matplotlib numpy shapely

```

* * * * *

1\. Robot Kinematic Simulation
------------------------------

### Overview

Simulates the motion of a robot using a bicycle kinematic model. It takes velocity and steering angle inputs from a CSV file and visualizes the robot's movement on a 2D plane.

### How to Run

1.  Prepare the input file `applied_input.csv` with velocity and steering angle data.
2.  Run the main script:

    ```
    python THE1/the1_main.py

    ```

### File Descriptions

-   `the1_main.py`: Defines the robot class and simulates the motion.
-   `the1_sim.py`: Handles visualization and animation of the robot's movement.
-   `the_hint.py`: Demonstrates coordinate transformations.

### Input Format (`applied_input.csv`)

```
Time;Velocity;Steering Angle
0;1.0;10
1;1.2;15
...

```

* * * * *

2\. Robot Path Planning Simulation
----------------------------------

### Overview

Simulates a robot navigating through an environment using path planning algorithms (Bug0, Bug1, Bug2) while avoiding obstacles.

### How to Run

1.  Navigate to the project directory:

    ```
    cd THE2

    ```

2.  Run the simulation:

    ```
    python the2.py

    ```

### Path Planning Algorithms

-   **Bug0**: Move towards the goal, follow obstacles, and find free paths.
-   **Bug1**: Move along the obstacle boundary and resume towards the goal.
-   **Bug2**: Efficiently navigate around obstacles using straight-line equations.

### Configuration

Change the path planner in `the2.py`:

```
robot = Robot(position=position, prev_position=prev_position, input=input_, memory=memory, planner=0)

```

-   `planner=0` → Bug0
-   `planner=1` → Bug1
-   `planner=2` → Bug2

* * * * *

3\. Search Algorithms on a Grid
-------------------------------

### Overview

Simulates various search algorithms (BFS, DFS, Dijkstra, A*) to find the shortest path from a start node to a goal node on a grid while avoiding obstacles. It uses `matplotlib` to visualize the process.

### How to Run

1.  Configure the grid environment with obstacles, start, and goal nodes in `THE3.py`.
2.  Run the desired search algorithm by modifying the `update()` function to use the chosen algorithm (e.g., `search_algorithm.bfs()`).

### Algorithms

-   **Breadth-First Search (BFS)**: Explores the grid level by level.
-   **Depth-First Search (DFS)**: Explores one path deeply before backtracking.
-   **Dijkstra**: Finds the shortest path based on cost.
-   **A***: Uses both cost and heuristic to find the best path.

### Visualization

Each node's color represents its state:

-   **Green**: Start node
-   **Red**: Goal node
-   **Black**: Obstacle
-   **Blue**: Visited node
-   **Path**: Green nodes along the path

* * * * *

License
-------

This project is for educational purposes. Feel free to modify and extend it.
