# Provides the window, drawing tools, mouse input and keyboard events.
import pygame

# A priority queue retrieves the lowest-priority-number item first.
from queue import PriorityQueue

# Create a square window using the same width and height in pixels.
WIDTH = 800
WIN = pygame.display.set_mode((WIDTH, WIDTH))
pygame.display.set_caption("A* Path Finding ALgorithm")

# RGB colors describe both how cells look and their current search state.
RED = (255,0,0)
GREEN = (0,255,0)
BLUE = (0,255,0)
YELLOW = (255,255,0)
WHITE = (255,255,255)
BLACK = (0,0,0)
PURPLE = (128,0,128)
ORANGE = (255,165,0)
GREY = (128,128,128)
TURQUOISE = (64,224,208)

# Represents a single square (node) in the pathfinding grid.
class Spot:

    # Initialize a node with grid indices, pixel coordinates, color and neighbor list.
    def __init__(self,row,col,width,total_rows):
        self.row = row
        self.col = col

        # Convert grid indices to pixel coordinates using the cell width.
        self.x = row * width
        self.y = col * width

        # White represents a cell that has not been assigned a search state.
        self.color = WHITE

        # Store accessible adjacent Spot objects; rebuild this list when needed.
        self.neighbors = []
        self.width = width
        self.total_rows = total_rows

    # Return the logical grid coordinates as a tuple rather than pixel coordinates.
    def get_pos(self):
        return self.row, self.col

    # Return True if this node has already been expanded (red).
    def is_closed(self):
        return self.color == RED

    # Return True if this node is marked as waiting to be explored (green).
    def is_open(self):
        return self.color == GREEN

    # Return True for a wall that cannot be traversed (black).
    def is_barrier(self):
        return self.color == BLACK

    # Check whether this node is marked as the starting point.
    def is_start(self):
        return self.color == ORANGE

    # Check whether this node is marked as the destination.
    def is_end(self):
        return self.color == TURQUOISE

    # Restore the default white appearance; this does not clear the neighbor list.
    def reset(self):
        # White represents a cell that has not been assigned a search state.
        self.color = WHITE

    # Mark the starting node orange.
    def make_start(self):
        self.color = ORANGE

    # Mark an expanded node red.
    def make_closed(self):
        self.color = RED

    # Mark a newly queued node green.
    def make_open(self):
        self.color = GREEN

    # Mark this node black so neighbor checks exclude it.
    def make_barrier(self):
        self.color = BLACK

    # Mark the destination turquoise.
    def make_end(self):
        self.color = TURQUOISE

    # Mark a node on the reconstructed route purple.
    def make_path(self):
        self.color = PURPLE

    # Draw this cell as a filled square on the supplied Pygame surface.
    def draw(self, win):
        pygame.draw.rect(win, self.color, (self.x, self.y, self.width, self.width))

    # Rebuild the list of traversable adjacent cells; diagonal movement is not allowed.
    def update_neighbors(self, grid):

        # Store accessible adjacent Spot objects; rebuild this list when needed.
        self.neighbors = []

        # Check the positive row direction, ensuring the index remains inside the grid.
        if self.row < self.total_rows - 1 and not grid[self.row + 1][self.col].is_barrier(): # DOWN
            self.neighbors.append(grid[self.row + 1][self.col])

        # Check the negative row direction without stepping outside the grid.
        if self.row > 0 and not grid[self.row - 1][self.col].is_barrier(): # UP
            self.neighbors.append(grid[self.row - 1][self.col])

        # Check the positive column direction and exclude walls.
        if self.col < self.total_rows - 1 and not grid[self.row][self.col + 1].is_barrier(): # RIGHT
            self.neighbors.append(grid[self.row][self.col + 1])

        # Intended to check the negative column direction. Note: this version checks row instead of col, so col - 1 can wrap around at the edge.
        if self.col > 0 and not grid[self.row][self.col - 1].is_barrier(): # LEFT
            self.neighbors.append(grid[self.row][self.col - 1])

    # Provide a less-than comparison for Spot objects; nodes themselves have no meaningful ordering.
    def __lt__(self, other):
        return False


# Calculate Manhattan distance: estimated remaining moves using only horizontal and vertical movement.
def h(p1,p2):

    # Unpack the two coordinate tuples into individual coordinates.
    x1, y1 = p1
    x2, y2 = p2
    # This reassignment has no effect on the result because x2 and y2 were already unpacked.
    p2 = (1,9)
    # Sum the absolute differences; obstacles are ignored by this estimate.
    return abs(x1 - x2) + abs(y1 - y2)

# Follow each node's recorded predecessor backward from the destination and animate the route.
def reconstruct_path(came_from, current, draw):
    # Continue until reaching a node with no recorded predecessor (normally the start).
    while current in came_from:
        # Move one step backward, then color that predecessor and refresh the display.
        current = came_from[current]
        current.make_path()
        draw()

# Run A* search using a drawing callback, grid, starting Spot and destination Spot.
def algorithm(draw, grid, start, end):
    # Use an insertion counter to break ties when two queued nodes have equal priority.
    count = 0
    # The frontier contains tuples of (priority, insertion order, Spot).
    open_set = PriorityQueue()
    # Begin the search by placing the start node in the frontier.
    open_set.put((0, count, start))
    # Map each discovered node to the predecessor on its best known route.
    came_from = {}
    # g(n): best known cost from the start; infinity means no route has been found yet.
    g_score = {spot: float("inf") for row in grid for spot in row}
    # The starting cell requires zero movement to reach.
    g_score[start] = 0
    # f(n): estimated total route cost, calculated as g(n) + h(n).
    f_score = {spot: float("inf") for row in grid for spot in row}
    # Initialize the estimated total cost for the starting node.
    f_score[start] = h(start.get_pos(), end.get_pos())

    # Track queued nodes in a set for fast membership checks.
    open_set_hash = {start}

    # Keep exploring until the frontier is exhausted or the destination is found.
    while not open_set.empty():
        # Process pending window events, including close requests.
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()

        # Remove the lowest-priority tuple and extract its Spot (element 2).
        current = open_set.get()[2]
        # Remove the selected node from the membership set because it is no longer queued.
        open_set_hash.remove(current)

        # Stop the search when the destination is reached and reconstruct the route.
        if current == end: # make path
            reconstruct_path(came_from, end, draw)
            return True

        # Examine each traversable adjacent node.
        for neighbor in current.neighbors:
            # Moving from the current cell to its neighbor costs one step.
            temp_g_score = g_score[current] + 1

            # Only update this neighbor when the new route is cheaper than its previous best route.
            if temp_g_score < g_score[neighbor]:
                # Record the current node as the neighbor's best known predecessor.
                came_from[neighbor] = current
                # Save the improved cost from the start.
                g_score[neighbor] = temp_g_score
                # Add the estimated remaining distance to the newly discovered cost.
                f_score[neighbor] = temp_g_score + h(neighbor.get_pos(), end.get_pos())
                # Queue only nodes not already waiting. Existing heap entries are not reprioritized by this implementation.
                if neighbor not in open_set_hash:
                    # Assign the next insertion order for priority ties.
                    count += 1
                    # Insert the neighbor using its estimated total route cost as priority.
                    open_set.put((f_score[neighbor], count, neighbor))
                    # Keep the membership set synchronized with the new queue entry.
                    open_set_hash.add(neighbor)
                    # Color the newly queued neighbor green.
                    neighbor.make_open()
        draw()

        # Mark the expanded node red, except for the starting node.
        if current != start:
            current.make_closed()

# Create and return a square two-dimensional list of Spot objects.
def make_grid(rows, width):
    grid = []
    # Calculate the integer pixel size of each cell.
    gap = width // rows
    # Iterate through grid indices.
    for i in range(rows):
        # Create an empty inner list for this grid index.
        grid.append([])
        for j in range(rows):
            # Construct an individual cell and append it to the grid.
            spot = Spot(i, j, gap, rows)
            grid[i].append(spot)

    return grid

# Overlay grey horizontal and vertical grid lines on the window.
def draw_grid(win, rows, width):
    # Calculate the integer pixel size of each cell.
    gap = width // rows
    # Iterate through grid indices.
    for i in range(rows):
        pygame.draw.line(win, GREY, (0, i * gap), (width, i * gap))
        for j in range(rows):
            pygame.draw.line(win, GREY, (j * gap, 0), (j * gap,width))


# Redraw the complete grid and present the updated frame on screen.
def draw(win, grid, rows, width):
    # Clear the previous frame before drawing the current cell colors.
    win.fill(WHITE)

    # Visit each inner list in the grid.
    for row in grid:
        # Visit every Spot within this inner list.
        for spot in row:
            spot.draw(win)
    # Overlay grid lines after filling the cells so the boundaries remain visible.
    draw_grid(win, rows, width)
    # Present the drawn surface so changes become visible to the user.
    pygame.display.update()



# Convert a mouse position in pixels into grid indices using integer division.
def get_clicked_pos(pos, rows, width):
    # Calculate the integer pixel size of each cell.
    gap = width // rows
    # Pygame returns screen (x, y); these local names are swapped to match this program's indexing.
    y, x = pos

    row = y // gap
    col = x // gap
    return row, col



# Initialize the grid and run the interactive Pygame event loop.
def main(win, width):
    # Use 50 cells per side (2,500 nodes total).
    ROWS = 50
    grid = make_grid(ROWS, width)

    # No starting cell is selected initially (or after clearing).
    start = None
    # No destination cell is selected initially (or after clearing).
    end = None

    run = True
    # This flag is initialized but is not used elsewhere in this version.
    started = False
    # Keep refreshing the window and processing input until the user closes it.
    while run:
        draw(win, grid, ROWS, width)
        # Process pending window events, including close requests.
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                run = False


            # Left click selects the start, then the destination, then creates walls.
            if pygame.mouse.get_pressed()[0]: # Left
                # Read the mouse's current pixel position.
                pos = pygame.mouse.get_pos()
                row, col = get_clicked_pos(pos, ROWS, width)
                # Use the converted indices to retrieve the selected Spot object.
                spot = grid[row][col]
                # The first eligible click assigns the start.
                if not start and spot != end:
                    start = spot
                    start.make_start()

                # The next eligible click assigns the destination.
                elif not end and spot != start:
                    end = spot
                    # Restore the destination's turquoise color after path reconstruction.
                    end.make_end()

                # After both endpoints exist, mark other clicked cells as barriers.
                elif spot  != end and spot != start:
                    spot.make_barrier()

            # Right click erases the selected cell and clears any associated endpoint reference.
            elif pygame.mouse.get_pressed()[2]: # Right
                # Read the mouse's current pixel position.
                pos = pygame.mouse.get_pos()
                row, col = get_clicked_pos(pos, ROWS, width)
                # Use the converted indices to retrieve the selected Spot object.
                spot = grid[row][col]
                spot.reset()
                # Clear the start reference if its cell was erased.
                if spot == start:
                    # No starting cell is selected initially (or after clearing).
                    start = None
                # After both endpoints exist, mark other clicked cells as barriers.
                elif spot == end:
                    # No destination cell is selected initially (or after clearing).
                    end = None

            # React to keyboard presses rather than continuously held keys.
            if event.type == pygame.KEYDOWN:
                # Space starts A* only after both endpoints have been selected.
                if event.key == pygame.K_SPACE and start and end:
                    # Visit each inner list in the grid.
                    for row in grid:
                        # Visit every Spot within this inner list.
                        for spot in row:
                            # Refresh traversable neighbors to reflect any walls added or removed.
                            spot.update_neighbors(grid)
                    # Pass a zero-argument lambda so A* can redraw using the current window and grid.
                    algorithm(lambda: draw(win, grid, ROWS, width), grid, start, end)

                # C clears the endpoints and replaces the entire grid with fresh cells.
                if event.key == pygame.K_c:
                    # No starting cell is selected initially (or after clearing).
                    start = None
                    # No destination cell is selected initially (or after clearing).
                    end = None
                    grid = make_grid(ROWS, width)

    pygame.quit()

# Start the application immediately when this module is executed or imported.
main(WIN, WIDTH)
