import numpy as np
from collections import deque
from .graph import Cell
from .utils import trace_path

"""
General graph search instructions:

First, define the correct data type to keep track of your visited cells
and add the start cell to it. If you need to initialize any properties
of the start cell, do that too.

Next, implement the graph search function. When you find a path, use the
trace_path() function to return a path given the goal cell and the graph. You
must have kept track of the parent of each node correctly and have implemented
the graph.get_parent() function for this to work. If you do not find a path,
return an empty list.

To visualize which cells are visited in the navigation webapp, save each
visited cell in the list in the graph class as follows:
     graph.visited_cells.append(Cell(cell_i, cell_j))
where cell_i and cell_j are the cell indices of the visited cell you want to
visualize.
"""


def depth_first_search(graph, start, goal):
    """Depth First Search (DFS) algorithm. This algorithm is optional for P3.
    Args:
        graph: The graph class.
        start: Start cell as a Cell object.
        goal: Goal cell as a Cell object.
    """
    graph.init_graph()  # Make sure all the node values are reset.

    """TODO (P3): Implement DFS (optional)."""

    # If no path was found, return an empty list.
    return []


def breadth_first_search(graph, start, goal):
    """Breadth First Search (BFS) algorithm.
    Args:
        graph: The graph class.
        start: Start cell as a Cell object.
        goal: Goal cell as a Cell object.
    """
    graph.init_graph()  # Make sure all the node values are reset.

    # If start is the same as goal, return path with just the start cell
    if start.i == goal.i and start.j == goal.j:
        return [Cell(start.i, start.j)]

    # Initialize BFS structures
    queue = deque([start])  # Queue for BFS (FIFO)
    visited = set()  # Set to track visited cells
    visited.add((start.i, start.j))

    # Mark start cell as visited for visualization
    graph.visited_cells.append(Cell(start.i, start.j))

    while queue:
        current = queue.popleft()

        # Check if we've reached the goal
        if current.i == goal.i and current.j == goal.j:
            # Found the goal! Trace back the path
            return trace_path(current, graph)

        # Explore neighbors
        neighbors = graph.find_neighbors(current.i, current.j)
        for ni, nj in neighbors:
            if (ni, nj) not in visited:
                visited.add((ni, nj))
                # Set parent relationship
                graph.parents[(ni, nj)] = (current.i, current.j)
                # Add to queue
                queue.append(Cell(ni, nj))
                # Add to visited cells for visualization
                graph.visited_cells.append(Cell(ni, nj))

    # If we get here, no path was found
    return []


def a_star_search(graph, start, goal):
    """A* Search (BFS) algorithm.
    Args:
        graph: The graph class.
        start: Start cell as a Cell object.
        goal: Goal cell as a Cell object.
    """
    graph.init_graph()  # Make sure all the node values are reset.

    """TODO (P3): Implement A*."""

    # If no path was found, return an empty list.
    return []
