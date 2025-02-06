import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation
import time
import heapq
from queue import PriorityQueue

class Node:
    def __init__(self, node_id, color='white'):
        self.id = node_id            # Unique ID for the node
        self.color = color           # Color representing the state of the grid
        self.neighbors = []          # Neighbors of the node
        self.is_visited = False      # Boolean indicating whether the node has been visited
        self.backpointer = None      # Parent node to trace the path (used in BFS)
        self.distance = float('inf') # Distance for Dijkstra and A* algorithms

    def add_neighbor(self, neighbor_node):
        # Only add neighbor if the node and the neighbor are not obstacles
        if neighbor_node and neighbor_node not in self.neighbors and neighbor_node.color != 'black':
            self.neighbors.append(neighbor_node)
            #neighbor_node.neighbors.append(self)  # Add this node as a neighbor to the other node

    def __lt__(self, other):
        # Compare nodes based on their distance for priority queue operations
        return self.distance < other.distance

class GridWorld:
    def __init__(self, rows, cols, obstacles_ids, start_id, goal_id):
        self.rows = rows
        self.cols = cols
        self.grid = []
        self.start = None
        self.goal = None
        self.obstacles_ids = obstacles_ids
        self.start_id = start_id
        self.goal_id = goal_id
        self.generate_grid()
        self.assign_neighbors()

    def generate_grid(self):
        """
        Generates the grid with specified obstacles, start, and goal positions.
        """
        node_id = 0
        for row in range(self.rows):
            row_nodes = []
            for col in range(self.cols):
                # Determine the color of the grid based on its ID
                if node_id in self.obstacles_ids:
                    color = 'black'  # Obstacle
                elif node_id == self.start_id:
                    color = 'green'  # Start node
                elif node_id == self.goal_id:
                    color = 'red'  # Goal node
                else:
                    color = 'white'  # Free space
                row_nodes.append(Node(node_id, color))
                node_id += 1
            self.grid.append(row_nodes)

        # Assign start and goal nodes
        self.start = self.get_node_by_id(self.start_id)
        self.goal = self.get_node_by_id(self.goal_id)

    def get_node_by_id(self, node_id):
        for row in self.grid:
            for node in row:
                if node.id == node_id:
                    return node
        return None

    def assign_neighbors(self):
        """
        Assigns neighbors (top, bottom, left, right) to each node, ensuring bidirectional relationships.
        Prints the node ID and its neighbors' IDs.
        """
        for row in range(self.rows):
            for col in range(self.cols):
                node = self.grid[row][col]
                if node.color == 'black':
                    continue  # Skip obstacles

                # Bottom neighbor
                if row > 0 and self.grid[row - 1][col].color != 'black':
                    node.add_neighbor(self.grid[row - 1][col])

                # Left neighbor
                if col > 0 and self.grid[row][col - 1].color != 'black':
                    node.add_neighbor(self.grid[row][col - 1])

                # Top neighbor
                if row < self.rows - 1 and self.grid[row + 1][col].color != 'black':
                    node.add_neighbor(self.grid[row + 1][col])

                # Right neighbor
                if col < self.cols - 1 and self.grid[row][col + 1].color != 'black':
                    node.add_neighbor(self.grid[row][col + 1])


    def update_grid(self):
        """
        Updates the grid for animation.
        """
        grid_matrix = np.zeros((self.rows, self.cols, 3))  # RGB values
        for row in range(self.rows):
            for col in range(self.cols):
                color = self.grid[row][col].color
                if color == 'black':
                    grid_matrix[self.rows - 1 - row, col] = [0, 0, 0]  # Black
                elif color == 'green':
                    grid_matrix[self.rows - 1 - row, col] = [0, 1, 0]  # Green
                elif color == 'red':
                    grid_matrix[self.rows - 1 - row, col] = [1, 0, 0]  # Red
                elif color == 'blue':
                    grid_matrix[self.rows - 1 - row, col] = [0.5, 0.5, 0.75]  # Bluish-grey
                elif color == 'path':
                    grid_matrix[self.rows - 1 - row, col] = [0, 1, 0]  # Path (green)
                else:
                    grid_matrix[self.rows - 1 - row, col] = [1, 1, 1]  # White
        return grid_matrix
    
    def get_all_nodes(self):
        """
        Returns a flat list of all nodes in the grid.
        """
        return [node for row in self.grid for node in row]

class SearchAlgorithms:
    def __init__(self, grid_world):
        self.grid_world = grid_world
        self.queue = []  # Queue for BFS
        self.stack = []  # Stack for DFS
        self.priority_queue = [] # Priority Queue for Dijkstra and A*
        self.visited_nodes = []  # List of all visited nodes
        self.found_path = False  # Flag to indicate if the goal has been reached
        self.path = []  # To store the backtracked path
        self.visited_count = 0  # Counter for visited grids
        self.start_time = None  # For tracking solution time


    def enqueue(self, node):
        self.queue.append(node)

    def dequeue(self):
        if self.queue:
            return self.queue.pop(0)
        return None

    def push(self, node):
        self.stack.append(node)

    def pop(self):
        if self.stack:
            return self.stack.pop()
        return None

    def enqueue_p(self, cost, node):
        heapq.heappush(self.priority_queue, (cost, node))  # Enqueue with priority (cost)

    def dequeue_p(self):
        if self.priority_queue:
            return heapq.heappop(self.priority_queue)  # Dequeue with the smallest cost
        return None



    def bfs(self):
        
        self.enqueue(self.grid_world.start)
        self.start_time = time.time()  # Start the timer for BFS

        while self.queue:
            current_node = self.dequeue()

            # Track the number of visited grids
            self.visited_count += 1

            if current_node == self.grid_world.goal:
                self.found_path = True
                self.trace_path(current_node)
                break

            current_node.is_visited = True
            if current_node.color != 'green':
                current_node.color = 'blue'

            for neighbor in current_node.neighbors:
                if not neighbor.is_visited and neighbor.color != 'black':
                    neighbor.is_visited = True
                    neighbor.backpointer = current_node
                    self.enqueue(neighbor)
            yield current_node

        # After BFS finishes, print the results
        if self.found_path == False:
            print("PATH DOES NOT EXIST.")
        else:
            # Solution time
            solution_time = time.time() - self.start_time
            print(f"Solution Time: {solution_time:.4f} seconds")

            # Number of visited grids
            print(f"Number of Visited Grids: {self.visited_count}")

            # Path length (count the nodes in the path)
            path_length = 0
            current_node = self.grid_world.goal
            while current_node.backpointer is not None:
                path_length += 1
                current_node = current_node.backpointer
            print(f"Path Length: {path_length}")
            
            
            
    def dfs(self):
        """
        Perform Depth First Search algorithm and update the grid at each step.
        """
        self.push(self.grid_world.start)
        self.start_time = time.time()  # Start the timer for DFS

        while self.stack:
            current_node = self.pop()

            # Track the number of visited grids
            self.visited_count += 1

            if current_node == self.grid_world.goal:
                self.found_path = True
                self.trace_path(current_node)
                break

            current_node.is_visited = True
            if current_node.color != 'green':
                current_node.color = 'blue'

            for neighbor in current_node.neighbors:
                if not neighbor.is_visited and neighbor.color != 'black':
                    neighbor.is_visited = True
                    neighbor.backpointer = current_node
                    self.push(neighbor)
            yield current_node

        # After DFS finishes, print the results
        if self.found_path == False:
            print("PATH DOES NOT EXIST.")
        else:
            # Solution time
            solution_time = time.time() - self.start_time
            print(f"Solution Time: {solution_time:.4f} seconds")

            # Number of visited grids
            print(f"Number of Visited Grids: {self.visited_count}")

            # Path length (count the nodes in the path)
            path_length = 0
            current_node = self.grid_world.goal
            while current_node.backpointer is not None:
                path_length += 1
                current_node = current_node.backpointer
            print(f"Path Length: {path_length}")
    
    def dijkstra(self):
        """
        Perform Dijkstra's algorithm and update the grid at each step.
        """
        self.start_time = time.time()  # Start time for measuring solution time
        self.enqueue_p(0, self.grid_world.start)  # Start node is enqueued with cost 0
        costs = {self.grid_world.start: 0}  # Dictionary to track costs to each node
        visited_nodes = set()  # Set to keep track of visited nodes
        self.visited_count = 0  # Initialize visited node count

        while self.priority_queue:
            cost, current_node = self.dequeue_p()  # Dequeue the node with the lowest cost

            if current_node in visited_nodes:  # Skip already visited nodes
                continue
            visited_nodes.add(current_node)
            self.visited_count += 1  # Increment visited count

            # Check if the goal node is reached
            if current_node == self.grid_world.goal:
                self.found_path = True
                yield from self.trace_path(current_node)  # Trace the path to the goal
                break

            # Mark the current node as visited
            current_node.is_visited = True
            if current_node.color != 'green':  # Avoid overwriting start or goal node color
                current_node.color = 'blue'

            # Explore neighbors
            for neighbor in current_node.neighbors:
                if neighbor in visited_nodes or neighbor.color == 'black':  # Skip visited or blocked nodes
                    continue

                new_cost = costs[current_node] + 1  # Assuming uniform cost of 1
                if neighbor not in costs or new_cost < costs[neighbor]:  # Update cost if it's better
                    costs[neighbor] = new_cost
                    neighbor.backpointer = current_node  # Update backpointer for path tracing
                    self.enqueue_p(new_cost, neighbor)

            yield current_node  # Yield the current node for visualization updates

        # If the loop ends without finding a path
        if not self.found_path:
            print("PATH DOES NOT EXIST.")
        else:
            self._print_results()

    def astar(self):
        """
        A* Search Algorithm
        This function uses a custom priority queue without heapq.
        """
        start_node = self.grid_world.start
        goal_node = self.grid_world.goal
        open_list = [(0, start_node)]  
        g_costs = {start_node: 0} 
        h_costs = {start_node: self.heuristic(start_node, goal_node)}  
        f_costs = {start_node: h_costs[start_node]}  
        while open_list:
            open_list.sort(key=lambda x: x[0])
            current_f, current_node = open_list.pop(0)
            if current_node == goal_node:
                self.found_path = True
                yield from self.trace_path(current_node)
                return
            current_node.is_visited = True
            if current_node.color != 'green':
                current_node.color = 'blue'
            for neighbor in current_node.neighbors:
                if neighbor.color == 'black' or neighbor.is_visited:
                    continue  
                tentative_g = g_costs[current_node] + 1
                if neighbor not in g_costs or tentative_g < g_costs[neighbor]:
                    g_costs[neighbor] = tentative_g
                    h_costs[neighbor] = self.heuristic(neighbor, goal_node)
                    f_costs[neighbor] = tentative_g + h_costs[neighbor]
                    neighbor.backpointer = current_node
                    open_list.append((f_costs[neighbor], neighbor))
            yield current_node
        
        # If the loop ends without finding a path
        if not self.found_path:
            print("PATH DOES NOT EXIST.")
        else:
           self._print_results()
           
    def _print_results(self):
        """
        Print the results after search completion.
        """
        if not self.found_path:
            print("PATH DOES NOT EXIST.")
            return

        # Solution time
        solution_time = time.time() - self.start_time
        print(f"Solution Time: {solution_time:.4f} seconds")

        # Number of visited grids
        print(f"Number of Visited Grids: {self.visited_count}")

        # Path length
        path_length = 0
        current_node = self.grid_world.goal
        while current_node.backpointer is not None:
            path_length += 1
            current_node = current_node.backpointer
        print(f"Path Length: {path_length}")

    def heuristic(self, node, goal):
        """
        Heuristic function for A*. Here, we use Manhattan distance as the heuristic.
        """
        node_row, node_col = divmod(node.id, self.grid_world.cols)
        goal_row, goal_col = divmod(goal.id, self.grid_world.cols)
        return abs(node_row - goal_row) + abs(node_col - goal_col)  # Manhattan distance


    def trace_path(self, node):
        """
        Backtrack from the goal to the start and mark the path as green incrementally.
        """
        current = node
        while current.backpointer:
            current = current.backpointer
            self.path.append(current)  # Store nodes in the path
            current.color = 'path'  # Mark the node as part of the path
            self.grid_world.update_grid()  # Update the grid after marking the path
            yield current  # Yield each backtracked node for animation




if __name__ == "__main__":
    rows, cols = 20, 20  # Define grid size

    #########################################  ENVIRONMENT 1 ###########################################
    #print("Running for Environment 1")
    #obstacles = [302,303,304,305,306,307,287,267,247,227,202,203,204,205,206,207,153,154,155,156,157,
    #             133,113,93,73,53,33,34,35,36,37]  # Define obstacle IDs
    #start_id = 265  # Start node ID
    #goal_id =  75  # Goal node ID

    #########################################  ENVIRONMENT 2 ###########################################
    #print("Running for Environment 2")
    #obstacles = [370,371,372,373,374,375,376,377,353,322,323,324,325,326,327,328,329,330,333,337,313,
    #             317,293,297,273,277,250,253,254,255,256,257,230,210,217,218,219,190,162,163,164,165,
    #             166,167,170,171,172,173,174,175,176,142,147,122,127,102,107,82,87,62,67,42,47]  # Define obstacle IDs
    #start_id = 125  # Start node ID
    #goal_id = 315  # Goal node ID

    #########################################  ENVIRONMENT 3 ###########################################
    print("Running for Environment 3")
    obstacles = [388,392,360,361,364,366,368,370,372,374,378,344,348,354,355,356,358,322,324,325,326,
                 327,328,329,330,332,336,338,302,310,312,316,318,304,282,288,284,264,244,290,292,293,
                 294,296,298,262,266,268,270,274,276,278,279,242,246,248,252,256,258,222,226,228,229,
                 230,231,232,233,234,235,236,237,238,202,206,218,181,182,183,184,185,186,187,188,189,
                 191,194,161,164,166,171,174,175,176,178,141,142,144,146,148,149,151,154,158,159,124,
                 126,129,131,134,136,137,138,100,101,102,103,104,106,107,108,109,111,114,118,91,94,96,
                 98,61,62,63,64,65,71,72,73,74,75,76,77,78,41,45,54,21,22,23,25,26,27,28,29,30,31,32,
                 34,35,36,37,38,5]  # Define obstacle IDs
    start_id = 67  # Start node ID
    goal_id = 257  # Goal node ID


    grid_world = GridWorld(rows, cols, obstacles, start_id, goal_id)
    search_algorithm = SearchAlgorithms(grid_world)
    


    # Set up the plot for animation
    fig, ax = plt.subplots(figsize=(10, 10))
    ax.set_xticks(np.arange(-0.5, grid_world.cols, 1), [])
    ax.set_yticks(np.arange(-0.5, grid_world.rows, 1), [])
    ax.grid(color='gray', linestyle='-', linewidth=0.5)

    # Initial plot
    im = ax.imshow(grid_world.update_grid(), origin='upper')

    def update(frame):
        """
        This function will be called in each frame of the animation.
        It performs one step of BFS and updates the grid.
        """
        current_node = next(search_algorithm.dijkstra())  # Get the next BFS step                                            #### !!!!!!!!!!! You should change the algorithm name here
        im.set_array(grid_world.update_grid())  # Update the grid with the new node color
        return [im]

    # Create the animation
    ani = FuncAnimation(fig, update, frames=search_algorithm.dijkstra(), interval=20, repeat=False, cache_frame_data=False)  #### !!!!!!!!!!! You should change the algorithm name here

    # Once animation is done, update grid with the path
    ani.event_source.stop()

    def final_update(*args):
        """
        This function will run after the animation to show the backtracked path.
        """
        if search_algorithm.found_path:
            # After BFS finishes, trace and display the backtracked path
            list(search_algorithm.trace_path(search_algorithm.grid_world.goal))  # Run the trace_path to update grid
            im.set_array(grid_world.update_grid())  # Update the grid to include the path
            plt.draw()  # Redraw the figure to show the final path

        
        #plt.close(fig)

    # Add callback for final update to show the backtracked path
    search_algorithm.found_path=True
    ani.event_source.add_callback(final_update)

    # Show the animation
    try:
        # Start the animation
        plt.show()
    except KeyboardInterrupt:
        print("Animation interrupted. Closing figure.")
        plt.close(fig)
        
    
    