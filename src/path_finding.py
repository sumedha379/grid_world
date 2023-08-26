from pathlib import Path
from queue import PriorityQueue
from typing import Set, Tuple, List

import numpy as np
import numpy.typing as npt
from matplotlib import pyplot as plt
import sys
import math


from src.utils import neighbors, plot_GVD, PathPlanMode, distance


def cell_to_GVD_gradient_ascent(
    grid: npt.ArrayLike, GVD: Set[Tuple[int, int]], cell: Tuple[int, int]
) -> List[Tuple[int, int]]:
    """Find the shortest path from any cell in the enviroment to a cell on the
    GVD using gradient ascent.
    Args:
        grid (numpy): NxN numpy array representing the world, with obstacles,
        walls, and the distance from each cell to the obstacles.
        GVD (set[tuple]): A set of tuples containing the cells on the GVD.
        cell (tuple): The starting/ending cell of this path.
    Returns:
        list<tuple>: list of tuples of the path.
    """

    path = [cell]
    s = cell
    while(s not in GVD):
        adjlist_s = neighbors(grid, s[0], s[1])
        max_neigh = None 
        max_adj_val = -1 * sys.maxsize
        for adj_s in adjlist_s:
            if grid[adj_s[0]][adj_s[1]] > max_adj_val :
                max_adj_val = grid[adj_s[0]][adj_s[1]] 
                max_neigh = adj_s
        path.append(max_neigh)
        s = max_neigh    
    path.append(s)    
    return path

def heuristic(A, B):
    return math.sqrt(math.pow(abs(A[0]-B[0]), 2.0) + math.pow(abs(A[1]-B[1]), 2.0))

def construct_path(reached, start, goal):
    s = goal
    path = [goal]
    while(s != start):
        path.append(reached[s]["parent"])
        s = reached[s]["parent"]
    return list(reversed(path))


def cell_to_GVD_a_star(
    grid: npt.ArrayLike, GVD: Set[Tuple[int, int]], cell: Tuple[int, int], 
    goal: Tuple[int, int]
) -> List[Tuple[int, int]]:
    """Find the shortest path from any cell in the enviroment to the GVD using
    A* with L2 distance heurstic.
    Args:
        grid (numpy): NxN numpy array representing the world, with obstacles,
        walls, and the distance from each cell to the obstacles.
        GVD (set<tuple>): A set of tuples containing the cells on the GVD.
        cell (tuple): The starting/ending cell of this path.
    Returns:
        list[tuple], dict, list[int]: list of tuples of the path, and the reached 
        dictionary, and the list of frontier sizes. 
    """

    # define a priority queue
    frontier = PriorityQueue()
    frontier.put((0, cell))
    frontier_size = [0]
    f, g, h = {}, {}, {}

    g[cell] = 0
    h[cell] = heuristic(cell, goal)
    f[cell] = g[cell] + h[cell]

    # construct a reached table using python dictionary. The key is the (x, y)
    # tuple of the cell position, the value is dictiionary with the cell's cost,
    # and cell parent.
    reached = {cell: {"cost": 0, "parent": None}}

    frontier_size.append(frontier.qsize())
    while not frontier.empty():
        f_val_s, s = frontier.get()
        if s in GVD:
            return construct_path(reached, cell, s), reached, frontier_size 
        adjlist_s = neighbors(grid, s[0], s[1])
        for adj_s in adjlist_s:
            if (adj_s not in g or g[s] + 1 < g[adj_s]):
                g[adj_s] = g[s] + 1 
                h[adj_s] = heuristic(adj_s, goal) 
                f[adj_s] = g[adj_s] + h[adj_s] 
                reached[adj_s] = {"cost": f[adj_s], "parent":s}
                frontier.put((f[adj_s], adj_s))
        frontier_size.append(frontier.qsize())
        

    # Implemented this to use the reached table (back pointers) to find
    # the path once reached a cell on the GVD.
    path = None
    return path, reached, frontier_size

def path_from_pointers(pointers, src, dest):
    s = dest
    path = [dest]
    while(s != src):
        path.append(pointers[s])
        s = pointers[s]
    return list(reversed(path))

def GVD_path(
    grid: npt.ArrayLike,
    GVD: Set[Tuple[int, int]],
    A: Tuple[int, int],
    B: Tuple[int, int],
    mode: PathPlanMode
) -> List[Tuple[int, int]]:
    """Find the shortest path between two points on the GVD using
    Breadth-First-Search
    Args:
        grid (numpy): NxN numpy array representing the world, with obstacles,
        walls, and the distance from each cell to the obstacles.
        A (tuple): The starting cell of the path.
        B (tuple): The ending cell of the path.
    Returns:
        list[tuple], dict, list[int]: return the path, pointers, and frontier 
        size array. 
    """
    # the set of cells on the GVD
    GVD = set(GVD)

    # the set of visited cells
    closed = set([])

    # the set of cells on the current frontier
    frontier = [A]

    # back pointers to find the path once reached the goal B. The keys
    # should both be tuples of cell positions (x, y)
    pointers = {}

    # the length of the frontier array, update this variable at each step. 
    frontier_size = [0]
    frontier_size.append(len(frontier))
    if mode == PathPlanMode.BFS:
        closed.add(A)

        while len(frontier) > 0:
            s = frontier.pop(0)
            adjlist_s = neighbors(grid, s[0], s[1])
            for adj_s in adjlist_s:
                if(adj_s not in closed and adj_s in GVD):
                    if(adj_s == B):
                        pointers[adj_s] = s 
                        frontier_size.append(len(frontier_size))
                        return path_from_pointers(pointers, A, B), pointers, frontier_size
                    frontier.append(adj_s)
                    closed.add(adj_s)
                    pointers[adj_s] = s 
            frontier_size.append(len(frontier_size))
    else:

        while len(frontier) > 0:
            s = frontier[-1] 
            frontier.pop()
            if s not in closed:
                closed.add(s)
            adjlist_s = neighbors(grid, s[0], s[1])
            for adj_s in adjlist_s:
                if(adj_s not in closed and adj_s in GVD):
                    if(adj_s == B):
                        pointers[adj_s] = s 
                        frontier_size.append(len(frontier_size))
                        return path_from_pointers(pointers, A, B), pointers, frontier_size
                    frontier.append(adj_s)
                    closed.add(adj_s)
                    pointers[adj_s] = s 
            frontier_size.append(len(frontier_size))

    return None, None, None


def compute_path(
    grid,
    GVD: set[tuple],
    start: tuple,
    goal: tuple,
    outmode: PathPlanMode = PathPlanMode.GRAD,
    inmode: PathPlanMode = PathPlanMode.DFS):

    """ Compute the path on the grid from start to goal using the methods
    implemented in this file. 
    Returns:
        list: a list of tuples represent the planned path. 
    """

    if outmode == PathPlanMode.GRAD:
        start_path = cell_to_GVD_gradient_ascent(grid, GVD, start)
        print(f"Start Path length: {len(start_path)} steps")
        end_path = list(reversed(cell_to_GVD_gradient_ascent(grid, GVD, goal)))
        print(f"End Path length: {len(end_path)} steps")
    else:
        start_path = cell_to_GVD_a_star(grid, GVD, start, goal)[0]
        end_path = list(reversed(cell_to_GVD_a_star(grid, GVD, goal, start)[0]))
    
    mid_path, reached, frontier_size = GVD_path(
        grid, GVD, start_path[-1], end_path[0], inmode)
    print(f"Mid Path length: {len(mid_path)} steps")

    return start_path + mid_path[1:-1] + end_path


def test_world(
    world_id, 
    start, 
    goal,
    outmode: PathPlanMode = PathPlanMode.GRAD,
    inmode: PathPlanMode = PathPlanMode.DFS,
    world_dir="worlds"):

    print(f"Testing world {world_id} with modes {inmode} and {outmode}")
    grid = np.load(f"{world_dir}/world_{world_id}.npy")
        
    GVD = set([tuple(cell) for cell in np.load(
        f"{world_dir}/world_{world_id}_gvd.npy")])
    # plot_GVD(grid, world_id, GVD)    
    path = compute_path(grid, GVD, start, goal, outmode=outmode, inmode=inmode)
    print(f"Path length: {len(path)} steps")
    plot_GVD(grid, world_id, GVD, path)