# Provides implementations for various replanning utilities
try:
    from flatland.core.transition_map import GridTransitionMap
    from flatland.envs.agent_utils import EnvAgent
    from flatland.utils.controller import get_action, Train_Actions, Directions, check_conflict, path_controller, evaluator, remote_evaluator
    from flatland.core.grid.grid_utils import IntVector2DArray, IntVector2D
except Exception as e:
    exit(1)




def delay_all_trains(existing_paths, current_timestep, malfunction):
    # function to delay a train by it's malfunction length
    # we basically repeat it's location for the malfunction length
    result = existing_paths.copy()
    try:
        result[current_timestep-1:current_timestep] = [existing_paths[current_timestep-1]] * (malfunction+2)
        return result
    except:
        return result

def slice_paths_by_timestep(paths, timestep, how='from'):
    # go through all paths, and slice the timestep if within index bounds
    # if how='to', then we slice to the timestep.
    # if how='from' then we slice from the timestep til the end of the array

    paths_sliced = []
    if how == 'from':

        for path in paths:
            try:
                path_sliced = path[timestep:]
            except:
                path_sliced = []
            paths_sliced.append(path_sliced)

    elif how == 'to' :
        for path in paths:
            try:
                path_sliced = path[:timestep]
            except:
                path_sliced = []
            paths_sliced.append(path_sliced)

    return paths_sliced