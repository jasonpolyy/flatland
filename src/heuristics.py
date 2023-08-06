# Implements manhattan and differential heuristics for the Flatland domain
# Functions for selecting pivots at random, by max manhattan distance
# Stores the differential heuristic dictionary in this module to be called globally

from dijkstra_search_rail import run_dijkstra
import random

try:
    from flatland.core.transition_map import GridTransitionMap
except Exception as e:
    exit(1)

diff_h_pivots = {}

def reset_pairwise_oracle():
    global diff_h_pivots
    diff_h_pivots = {}

def select_random_pivots(n_pivots, rail):
    # select random pivots
    pivots = []
    while len(pivots) < n_pivots:
        try:
        # we'll just initialise in random places
            loc = (random.randint(0, rail.height), random.randint(0, rail.width))

            # if traversible, add to the pivots list
            if rail.get_full_transitions(loc[0], loc[1]):
                pivots.append(loc)

        except:
            continue
    return pivots

def manhattan_distance(current_state, goal_state):
    """
    Define just manhattan distance
    """
    h = abs(current_state[0]-goal_state[0]) + abs(current_state[1]-goal_state[1])
    return h

def manhattan_heuristic(domain: GridTransitionMap, current_state, goal_state):
    """
    Manhattan distance heuristic
    """
    h = manhattan_distance(current_state, goal_state)
    return h

def select_pivots_by_distance(n_pivots, rail: GridTransitionMap):
    # select pivots by adding pivots that are maximally distanced from other selected pivots.
    # this chooses pivots that will be far away from each other in order to potentially increase search efficiency.
    pivots = []

    # we need to select any initial pivot. we'll just choose one randomly that is valid.
    while len(pivots) < 1:
        start_loc = (random.randint(0, rail.height-1), random.randint(0, rail.width-1))
        if rail.get_full_transitions(start_loc[0], start_loc[1]):
            pivots.append(start_loc)

    # while we haven't filled the pivots list out
    while len(pivots) < n_pivots:

        # the best point is one that maximises the manhattan distance between it and all the ones chosen so far
        best_point = max(((x, y) for x in range(rail.height) for y in range(rail.width) 
                         if (x, y) not in pivots and rail.get_full_transitions(x, y)),
                         key=lambda p: min(manhattan_distance((p[0], p[1]), (px, py)) for px, py in pivots))
        pivots.append(best_point)
    return pivots


def init_differential_heuristic_pivots(rail: GridTransitionMap, n_pivots: int =12, pivot_method='distance', reset=False):
    # initialises the differential heuristic pivots
    global diff_h_pivots

    # resets the differential heuristic
    if reset:
        diff_h_pivots = {}

    if not len(diff_h_pivots):

        if pivot_method == 'random':
        # create n number of pivots
            selected_pivots = select_random_pivots(n_pivots, rail)
        elif pivot_method == 'distance':
            selected_pivots = select_pivots_by_distance(n_pivots, rail)
            
        for pivot in selected_pivots:

            diff_h_pivots[pivot] = {}

        # run djikstra for each pivot
        for pivot in diff_h_pivots.keys():
        #pivot = (7,0)
            diff_h_pivots[pivot] = run_dijkstra(rail, pivot)

def differential_heuristic(domain: GridTransitionMap, current_state, goal_state):
    # implements a differential heuristic
    # assumes that diff_h has been initialised already

    global diff_h_pivots

    if not len(diff_h_pivots):
        raise Exception("diff_h_pivots has not been initialised yet. Please run init_differential_heuristic_pivots.")

    all_heuristics = []
    for pivot in diff_h_pivots.keys():
        try:
            all_heuristics.append(abs(diff_h_pivots[pivot][current_state] - diff_h_pivots[pivot][goal_state]))
        except:
            continue

    # reduce to manhattan heuristic in case pivots all fails
    if len(all_heuristics) == 0:
        return manhattan_heuristic(domain, current_state, goal_state)
    else:
        return max(all_heuristics)

def pairwise_oracle(domain: GridTransitionMap, current_state, goal_state):

    # here we calculate the true distance to the objective
    global diff_h_pivots

    # look up true distance from 
    if goal_state not in diff_h_pivots:
        diff_h_pivots[goal_state] = run_dijkstra(domain, goal_state)

    # look up the true distance
    return diff_h_pivots[goal_state][current_state]
