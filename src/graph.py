import os
import numpy as np


class Cell(object):
    def __init__(self, i, j):
        self.i = i  # x-axis index (column)
        self.j = j  # y-axis index (row)


"""TODO: You may consider defining a class to store your node data. If so, do
that here."""


class GridGraph:
    """Helper class to represent an occupancy grid map as a graph."""
    def __init__(self, file_path=None, width=-1, height=-1, origin=(0, 0),
                 meters_per_cell=0, cell_odds=None, collision_radius=0.15, threshold=-100):
        """Constructor for the GridGraph class.

        Args:
            file_path: Path to the map file to load. If provided, all other map
                       properties are loaded from this file.
            width: Map width in cells.
            height: Map height in cells.
            origin: The (x, y) coordinates in meters of the cell (0, 0).
            meters_per_cell: Size in meters of one cell.
            cell_odds: List of length width * height containing the odds each
                       cell is occupied, in range [127, -128]. High values are
                       more likely to be occupied.
            collision_radius: The radius to consider when computing collisions in meters.
            threshold: Cells above this value are considered to be in collision.
        """
        if file_path is not None:
            # If the file is provided, load the map.
            assert self.load_from_file(file_path)
        else:
            self.width = width
            self.height = height
            self.origin = origin
            self.meters_per_cell = meters_per_cell
            self.cell_odds = cell_odds

        self.threshold = threshold
        self.set_collision_radius(collision_radius)
        self.visited_cells = []  # Stores which cells have been visited in order for visualization.

        # Initialize parent tracking dictionary
        self.parents = {}  # Dictionary to store parent of each cell: {(i,j): (parent_i, parent_j)}

    def as_string(self):
        """Returns the map data as a string for visualization."""
        map_list = self.cell_odds.astype(str).tolist()
        rows = [' '.join(row) for row in map_list]
        cell_data = ' '.join(rows)
        header_data = f"{self.origin[0]} {self.origin[1]} {self.width} {self.height} {self.meters_per_cell}"
        return ' '.join([header_data, cell_data])

    def load_from_file(self, file_path):
        """Loads the map data from a file."""
        if not os.path.isfile(file_path):
            print(f'ERROR: loadFromFile: Failed to load from {file_path}')
            return False

        with open(file_path, 'r') as file:
            header = file.readline().split()
            origin_x, origin_y, self.width, self.height, self.meters_per_cell = map(float, header)
            self.origin = (origin_x, origin_y)
            self.width = int(self.width)
            self.height = int(self.height)

            # Check sanity of values.
            if self.width < 0 or self.height < 0 or self.meters_per_cell < 0.0:
                print('ERROR: loadFromFile: Incorrect parameters')
                return False

            # Reset odds list.
            self.cell_odds = np.zeros((self.height, self.width), dtype=np.int8)

            # Read in each cell value.
            for r in range(self.height):
                row = file.readline().strip().split()
                for c in range(self.width):
                    self.cell_odds[r, c] = np.int8(row[c])

        return True

    def pos_to_cell(self, x, y):
        """Converts a global position to the corresponding cell in the graph.
        Args:
            x: The global x position in meters.
            y: The global y position in meters.
        Returns:
            The cell coordinate in the graph.
        """
        i = int(np.floor((x - self.origin[0]) / self.meters_per_cell))
        j = int(np.floor((y - self.origin[1]) / self.meters_per_cell))

        return Cell(i, j)

    def cell_to_pos(self, i, j):
        """Converts a cell coordinate in the graph to the corresponding global position.
        Args:
            i: The x-axis index (column) of the cell in the graph.
            j: The y-axis index (row) index of the cell in the graph.
        Returns:
            A tuple containing the global position, (x, y)."""
        x = (i + 0.5) * self.meters_per_cell + self.origin[0]
        y = (j + 0.5) * self.meters_per_cell + self.origin[1]
        return x, y

    def is_cell_in_bounds(self, i, j):
        """Checks whether the provided cell is within the bounds of the graph."""
        return i >= 0 and i < self.width and j >= 0 and j < self.height

    def is_cell_occupied(self, i, j):
        """Checks whether the provided index in the graph is occupied (i.e. above
        the threshold.)"""
        return self.cell_odds[j, i] >= self.threshold

    def set_collision_radius(self, r):
        """Sets the collision radius and precomputes some values to help check
        for collisions.
        Args:
            r: The collision radius (meters).
        """
        r_cells = max(1, int(np.ceil(r / self.meters_per_cell)))  # Radius in cells, minimum 1.
        # Get all the indices in a mask covering the robot.
        r_indices, c_indices = np.indices((2 * r_cells - 1, 2 * r_cells - 1))
        c = r_cells - 1  # Center point of the mask.
        dists = (r_indices - c)**2 + (c_indices - c)**2  # Distances to the center point.
        # These are the indices which are in collision for the robot with this radius.
        self._coll_ind_j, self._coll_ind_i = np.nonzero(dists <= (r_cells - 1)**2)

        # Save the radius values.
        self.collision_radius = r
        self.collision_radius_cells = r_cells

    def check_collision(self, i, j):
        """Checks whether this cell is in collision based on the collision radius
        defined in the graph."""
        # We will use the previously calculated mask over the robot radius to
        # check whether any indices in a radius around the robot are in collision.
        j_inds = self._coll_ind_j + j - (self.collision_radius_cells - 1)
        i_inds = self._coll_ind_i + i - (self.collision_radius_cells - 1)

        # These are the indices in the bounds of the grid after we shift the
        # robot mask to the cell we are checking.
        in_bounds = np.bitwise_and(np.bitwise_and(j_inds >= 0, j_inds < self.height),
                                   np.bitwise_and(i_inds >= 0, i_inds < self.width))

        return np.any(self.is_cell_occupied(i_inds[in_bounds], j_inds[in_bounds]))

    def get_parent(self, cell):
        """Returns a Cell object representing the parent of the given cell, or
        None if the node has no parent. This function is used to trace back the
        path after graph search."""
        parent_coords = self.parents.get((cell.i, cell.j))
        if parent_coords is not None:
            return Cell(parent_coords[0], parent_coords[1])
        return None

    def init_graph(self):
        """Initializes the node data in the graph in preparation for graph search.

        When this funtion is called, the graph will have loaded the members
        which store the properties of the graph, like width, height, and cell
        odds values. You should use this information to initialize your added
        values, like the distances and the nodes."""
        self.visited_cells = []  # Reset visited cells for visualization.
        self.parents = {}  # Reset parent tracking dictionary

    def find_neighbors(self, i, j):
        """Returns a list of the neighbors of the given cell. This should not
        include any cells outside of the bounds of the graph."""
        nbrs = []
        # Check the 4 possible neighbors: up, down, left, right
        directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]  # (di, dj) pairs

        for di, dj in directions:
            ni, nj = i + di, j + dj
            if self.is_cell_in_bounds(ni, nj) and not self.check_collision(ni, nj):
                nbrs.append((ni, nj))

        return nbrs
