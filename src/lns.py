import random
import math
from typing import List, Tuple

try:
    from flatland.core.transition_map import GridTransitionMap
    from flatland.envs.agent_utils import EnvAgent
    from flatland.utils.controller import get_action, Train_Actions, Directions, check_conflict, path_controller, evaluator, remote_evaluator
except Exception as e:
    exit(1)

from lib_piglet.search.search_node import compare_node_f
from lib_piglet.utils.data_structure import bin_heap

from heuristics import manhattan_heuristic, differential_heuristic, init_differential_heuristic_pivots, pairwise_oracle
from sipp import SIPP


def determine_LNS_iterations(n_agents, map_size):
    # Simple empirically derived function to determine number of iterations for LNS 
    return math.ceil(250 * math.exp(-0.015*(n_agents + map_size)))

def find_collisions(paths):
    # find edge and vertex collisions between all paths and return the indicies for the collisions
    collisions = []
    max_t = max(len(p) for p in paths)
    for path in range(len(paths)): 
        curr_path = paths[path]
        other_paths = paths[:path] + paths[path+1:]
        for other_path in other_paths:
            conflict = False
            for t in range(0, max_t):
                try:
                    if (t < len(curr_path) and t < len(other_path)) and other_path[t] == curr_path[t]:
                        conflict = True
                    if  (t < len(curr_path) and t < len(other_path)) and other_path[t+1] == curr_path[t] and  other_path[t] ==curr_path[t+1]:
                        conflict = True
                    if conflict:
                        collisions.append(path)
                        break
                except:
                    continue
    return collisions

def LNS(agents: List[EnvAgent], rail: GridTransitionMap, existing_paths: List[Tuple], max_timestep: int, max_iterations=100, early_stop = 20, strategy=1) -> List[list]:
    """
    Run Large Neighbourhood Search in the Flatland domain for various EnvAgent agents.
    Terminates after max_iterations or if there are no improvements after early_stop iterations.

    Parameters
    ----------
    agents : List[EnvAgent]
        agents to have their paths planned
    rail : GridTransitionMap
        the domain
    existing_paths : List[Tuple]
        list of existing planned paths to be treated as obstacles
    max_timestep : int
        maximum timestep a path can have
    max_iterations : int, optional
        max iterations before termination, by default 1000
    early_stop : int, optional
        stops execution if no improvement after a number of iterations, by default 20
    strategy : int, optional
        which neighbourhood strategy to use. 1 for random, by default 1

    Returns
    -------
    List[list]
        paths for each agent
    """
    best_current_SIC = sum(len(p) for p in existing_paths) - len(existing_paths)
    penalty = sum(len(existing_paths[agent_id]) - 1 - agents[agent_id].deadline for agent_id in range(len(agents)) if len(existing_paths[agent_id]) - agents[agent_id].deadline >0)
    best_current_score = best_current_SIC + penalty
    iteration = 0
    no_improvement_counter = 0
    best_paths = existing_paths
    n_collisions = len(find_collisions(existing_paths))

    # want to minimise number of collisions (ideally zero but shit happens)
    while iteration < max_iterations:

        # if there has been no improvement to SIC after a while, stop LNS and return best so far
        if no_improvement_counter >= early_stop:
            break

        # randomly select agents
        if strategy == 1:
            n_agents = random.randint(0, len(agents))
            agent_ids = random.sample(range(0, len(agents)), n_agents)

        # destroy paths for the randomly selected agents
        existing_paths_empty = [[] if i in agent_ids else value for i, value in enumerate(best_paths)]
        agents_destroyed = [agents[i] for i in agent_ids]
        
        print(agent_ids)

        lns_plans = []
        num_empty_paths = 0
        for agent_id in agent_ids:

            start = agents[agent_id].initial_position
            start_direction = agents[agent_id].initial_direction
            goal = agents[agent_id].target

            open_list = bin_heap(compare_node_f)
            heuristic_function = differential_heuristic
            heuristic_weight = 1 

            # instantiated a Safe Interval Path Planner
            sipp = SIPP(rail, open_list, heuristic_function, heuristic_weight)

            # run SIPP for the agent
            path = sipp.get_path(start, start_direction, goal, agent_id, existing_paths_empty, max_timestep, start_collision_check=False)

            if not len(path):
                num_empty_paths += 1
            #existing_paths.append(path)
            lns_plans.append(path)
            existing_paths_empty[agent_id] = path

        # repair the destroyed agents paths
        all_paths = existing_paths_empty.copy()
        for i in range(len(agents_destroyed)):
            all_paths[agent_ids[i]] = lns_plans[i]
        
        #print(num_empty_paths)
        # calculate if path is the best so far
        curr_SIC = sum(len(p) for p in all_paths) - len(all_paths)
        curr_penalty = sum(len(all_paths[agent_id]) - 1 - agents[agent_id].deadline for agent_id in range(len(agents)) if len(all_paths[agent_id]) - agents[agent_id].deadline >0)
        curr_score = curr_SIC+curr_penalty
        curr_n_collisions = len(find_collisions(all_paths))

        # if score improved take it !
        if curr_score < best_current_score and curr_n_collisions <= n_collisions and len(agent_ids) == (len(lns_plans)-num_empty_paths):
            print(f"Current score {curr_score} improved on best score so far {best_current_score}")
            print(f"Current number of collisions {curr_n_collisions} against best {n_collisions}")
            best_current_score = curr_score
            best_paths = all_paths
            n_collisions = curr_n_collisions
            no_improvement_counter = 0
        else:
            no_improvement_counter += 1

        iteration+=1

    return best_paths
