README
======

This repository contains Python code for implementing and visualizing various search algorithms on a grid-based environment. The grid world is populated with obstacles, a start node, and a goal node. The search algorithms (BFS, DFS, Dijkstra, A*) are used to find the optimal path from the start node to the goal while avoiding obstacles. The code visualizes the grid and the search process using `matplotlib` animations.

Files
-----

### THE3.py

`THE3.py` implements a grid-based world and several search algorithms, including Breadth-First Search (BFS), Depth-First Search (DFS), Dijkstra, and A*.

#### Key Components:

1.  **Node Class**: Represents a cell in the grid, which can be an obstacle or free space. Each node has neighbors and properties like color (indicating state), distance, and a backpointer for path tracing.

2.  **GridWorld Class**: Creates the grid with start, goal, and obstacles. It also assigns neighbors to each node and updates the grid for visualization.

3.  **SearchAlgorithms Class**: Implements the BFS, DFS, Dijkstra, and A* algorithms. The search algorithms explore the grid and update the grid visualization during the process.

4.  **Animation**: The grid is updated in each frame using `matplotlib.animation.FuncAnimation`, where the nodes' colors represent different states (such as visited, path, or unvisited).

#### Algorithms:

-   **Breadth-First Search (BFS)**: Explores the grid level by level, ensuring the shortest path in terms of the number of steps.
-   **Depth-First Search (DFS)**: Explores a path deeply before backtracking, which might not always find the shortest path.
-   **Dijkstra**: Finds the shortest path based on the cumulative cost from the start node to the goal node, assuming uniform step costs.
-   **A***: Uses both the actual cost from the start and a heuristic (Manhattan distance) to estimate the best path towards the goal.

#### Visualization:

-   Each node's color indicates its state:
    -   **Green** for the start node
    -   **Red** for the goal node
    -   **Black** for obstacles
    -   **Blue** for visited nodes
    -   **Path** nodes are marked with a green color.

#### How to Use:

1.  **Grid Size and Environment Configuration**:

    -   Set the grid size (`rows` and `cols`) and define obstacles, start, and goal nodes.
    -   Three environment configurations are available, with specific obstacles, start, and goal locations.
2.  **Running the Search**:

    -   To execute a specific search algorithm, modify the `update` function in the animation section, replacing `search_algorithm.dijkstra()` with your algorithm of choice (e.g., `search_algorithm.bfs()`).
    -   After executing a search, the animation will visualize the algorithm's progress step-by-step.
3.  **Final Path Visualization**:

    -   Once the algorithm finds the path (if any), the path will be traced and highlighted in the final grid.

#### Example Execution:

To visualize the result for **Environment 3**, the following grid configuration is used:

-   **Rows**: 20
-   **Columns**: 20
-   **Obstacles**: A list of obstacle node IDs
-   **Start Node**: Node ID 67
-   **Goal Node**: Node ID 257

You can switch between the search algorithms by adjusting the `update()` function to match the algorithm you wish to visualize (e.g., `search_algorithm.dfs()`).

Requirements
------------

-   **Python 3.x**
-   **matplotlib**
-   **numpy**

Install the required libraries using `pip`:

```
pip install matplotlib numpy

```

License
-------

This project is open-source and free to use for educational and personal purposes.
