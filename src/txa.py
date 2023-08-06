# Classes to implement TXA* for the Flatland domain
# contains txa search nodes and the TXA algorithm

from lib_piglet.search.search_node import compare_node_f
from lib_piglet.utils.data_structure import bin_heap
from lib_piglet.constraints.grid_constraints import grid_reservation_table

from lib_piglet.expanders import graph_expander
from lib_piglet.utils.tools import eprint
import glob, os, sys

from heuristics import manhattan_heuristic, differential_heuristic, init_differential_heuristic_pivots, pairwise_oracle, reset_pairwise_oracle

#import necessary modules that this python scripts need.
try:
    from flatland.core.transition_map import GridTransitionMap
    from flatland.utils.controller import get_action, Train_Actions, Directions, check_conflict, path_controller, evaluator, remote_evaluator
except Exception as e:
    eprint("Cannot load flatland modules!", e)
    exit(1)

import bisect

safe_intervals_all = None

class txa_node:
    """
    Defines a TXA* node with time and interval instance variables.
    This is used specifically for the SIPP class.
    """
    def __init__(self):
        # some default values for uninitialised nodes
        self.action_: object = None
        self.state_: object = None
        self.parent_: object = None
        self.g_: float = 0
        self.depth_: int = 0
        self.instance_: int = 0
        
        self.h_: float = 0
        self.f_ : float= 0
        self.timestep_: int = 0

        self.t: int = 0
        self.closed_:bool = False
        self.open_handle_: object = None
        self.expanded: bool = False

    # Is the node closed
    # @return bool True if the node is closed
    def is_closed(self):
        return self.closed_

    # Mark the node as closed
    def close(self):
        self.closed_ = True

    # Mark the node as open
    def open(self):
        self.closed_ = False

    def __str__(self):
        #return str((self.state_, self.interval_, self.action_.move_, self.t))

        return str((self.state_, self.timestep_, self.action_.move_))

    def __repr__(self):
        return self.state_.__repr__()

        #return (self.state_, self.t).__repr__()

    def __eq__(self, other):
        
        if (other == None):
            return False
        return self.state_ == other.state_ and self.timestep_ == other.timestep_ and self.action_.move_ == other.action_.move_ 
        #return self.state_ == other.state_ and self.interval_[1] == other.interval_[1] and self.action_.move_ == other.action_.move_ 
        #return self.state_ == other.state_ 
        #return hash((self.state_, self.t)) == hash((other.state_, other.t))

    def __hash__(self):
        #return hash((self.state_)) 
        #return hash((self.state_, self.interval_[1], self.action_.move_)) 
        #return hash(self.state_) + hash( self.interval_) + hash(self.action_.move_)

        return hash((self.state_, self.timestep_, self.action_.move_)) 

class TXA():
    """
    Defines the Time Expanded A* algorithm
    This algorithm was built off of the piglet graph search algorithm in lib_piglet.search.graph_search
    """
    
    def __init__(self, domain: GridTransitionMap, open: bin_heap, heuristic_function, heuristic_weight=1, agent_id=-1, reset=False, reservation_table: grid_reservation_table = None):
        self.all_nodes_list_ = {}
        self.nodes_expanded_ = 0
        self.nodes_generated_ = 0
        self.rail = domain
        self.open_list_ = open
        self.agent_id = agent_id
        self.reservation_table = reservation_table

        self.reset = reset

        # if the heuristic function is a differential heuristic, initialise it
        if heuristic_function == differential_heuristic:
            init_differential_heuristic_pivots(domain, n_pivots=15, pivot_method='distance', reset=self.reset)
        elif heuristic_function == pairwise_oracle:
            reset_pairwise_oracle()

        self.heuristic_function_ = heuristic_function
        self.heuristic_weight_ = heuristic_weight

    def get_path(self, start: tuple, start_direction: int, goal: tuple, agent_id: int, existing_paths: list, max_timestep: int, start_collision_check = False):

        path = []
        #all_nodes_list_ = {}
        nodes_expanded_ = 0
        nodes_generated_ = 0
        status_ = "Failed"

        # implement a* here.
        #self.open_list_ = bin_heap(compare_node_f)
        start_node = txa_node()

        # Our state will be a list consisting of the location and the direction.
        start_state = start
        start_node.state_ = start_state
        start_node.action_ = graph_expander.graph_action(start_direction, 1)

        self.open_list_.push(start_node)
        self.all_nodes_list_[start_node] = start_node

        #print(f'start node {start_node}')

        # continue while there are still nods on OPEN
        while (len(self.open_list_) > 0):

            current: txa_node = self.open_list_.pop()
            current.close()
            nodes_expanded_ +=1

            #print(f'\tcurr node {current}')


            # goal example. if successful, return the solution
            if self.is_goal(current.state_, goal):
                path = self.solution_time(current)
                status_ = "Success"
                break

            loc_state = current.state_
            direction_state = current.action_.move_
            valid_transitions = self.rail.get_transitions(loc_state[0],loc_state[1],direction_state)
            valid_transitions_wait = list(valid_transitions)
            valid_transitions_wait.append(1)
            #valid_transitions.append(1)
            #print(f'\t{valid_transitions_wait}')

            # loop through all valid transitions
            for i in range(0,len(valid_transitions_wait)):
                
                # if valid, process the transition
                if valid_transitions_wait[i]:
                    new_x=loc_state[0]
                    new_y=loc_state[1]
                    action = i

                    if action == Directions.NORTH:
                        new_x -= 1
                    elif action == Directions.EAST:
                        new_y += 1
                    elif action == Directions.SOUTH:
                        new_x += 1
                    elif action == Directions.WEST:
                        new_y -= 1
                    elif i == 4:
                        None

                    loc_updated = (new_x,new_y)
                    direction_updated = action

                    if i==4:
                        direction_updated = current.action_.move_

                    t = current.timestep_

                    conflict = False
                    for p in existing_paths:
                        if t+1 < len(p) and p[t+1] == (new_x,new_y):
                            conflict = True
                        if t+1 < len(p) and p[t+1] ==(loc_state[0],loc_state[1]) and  p[t] ==(new_x,new_y):
                            conflict = True
                    if conflict:
                        #print('conflict')
                        continue
                    
                    # reuse the graph_action class 
                    cost =1
                    new_action = graph_expander.graph_action(direction_updated, cost)

                    # generate the successor node
                    #succ_node = self.generate(state=loc_updated, action=new_action, parent=current, goal_=goal, heuristic_function_ = manhattan_heuristic, heuristic_weight_=1)
                    succ_node = self.generate(state=loc_updated, action=new_action, parent=current, goal_=goal)
                    #print(f'\tnode generated {succ_node}')

                    # succ_node not in any list, add it to open list
                    if succ_node not in self.all_nodes_list_:
                        # we need this open_handle_ to update the node in open list in the future
                        #print('\tnode hasnt been explored yet')
                        succ_node.open_handle_ = self.open_list_.push(succ_node)
                        self.all_nodes_list_[succ_node] = succ_node
                        nodes_generated_+= 1

                        # succ_node only have the same hash and state comparing with the on in the all nodes list
                        # It's not the one in the all nodes list,  we need the real node in the all nodes list.
                    exist = self.all_nodes_list_[succ_node]
                    if not exist.is_closed():
                        self.relax(exist, succ_node)

        if len(path) >0:
            redo_path = []
            for i in range(len(path)):
                if i>0:
                    for _ in range(path[i].timestep_ - path[i-1].timestep_):
                        redo_path.append(path[i-1].state_)

                elif i == len(path):
                    redo_path.append(path[i].state_)
            
            # append the goal
            redo_path.append(goal)
            path = redo_path


        # check for collisions through start positions
        # if there is a collision, return an empty list for that agent
        if start_collision_check:
            try:
                start_location_path = path[0]
                last_t_at_start = self.start_position_last_t(path)
                for t in range(last_t_at_start):
                    if self.reservation_table.is_reserved(start_location_path, t, self.agent_id):
                        return []
            except:
                return path
               
        return path                 


    def start_position_last_t(self, path):
        # count how long the agent is in the start position for
            start_pos = path[0]

            for i in range(1, len(path)):
                if path[i] == start_pos:
                    continue
                else:
                    break  # stop checking the rest of the list

            return i-1

    def generate(self, state, action, parent: txa_node, goal_: tuple = None):

        retval = txa_node()
        retval.state_ = state
        retval.action_ = action
        if (parent == None):
            # initialise the node from scratch
            # NB: we usually do this only for the start node
            retval.g_ = 0
            retval.depth_ = 0
            retval.timestep_ = 0

            retval.t = 0
        else:
            # initialise the node based on its parent
            retval.g_ = parent.g_ + action.cost_
            retval.depth_ = parent.depth_ + 1
            retval.parent_ = parent
            retval.timestep_= parent.timestep_ + 1


        if self.heuristic_function_ is None:
            retval.h_ = 0
            retval.f_ = retval.g_
        else:
            retval.h_ = self.heuristic_function_(self.rail, retval.state_, goal_)
            retval.f_ = retval.g_ + retval.h_ * self.heuristic_weight_
        return retval

    def is_goal(self, current_state: tuple, goal: tuple):
        # check if the goal is found yet
        return current_state == goal

    def solution_time(self, goal_node: txa_node):
        # recurse from goal node to start node of the solution
        tmp = goal_node
        sol = []
        while (tmp != None):
            sol.append(tmp)
            tmp = tmp.parent_
            
        sol.reverse()

        return sol

    def relax(self, exist:txa_node, new:txa_node):
        if exist.g_ > new.g_:
            exist.f_ = new.f_
            exist.g_ = new.g_
            exist.depth_ = new.depth_
            exist.instance_ = new.instance_
            exist.action_ = new.action_
            exist.timestep_ = new.timestep_
            exist.h_ = new.h_
            exist.parent_ = new.parent_
            if exist.open_handle_ is not None:
                # If handle exist, we are using bin_heap. We need to tell bin_heap one element's value
                # is decreased. Bin_heap will update the heap to maintain priority structure.
                self.open_list_.decrease(exist.open_handle_)

    def reset_statistic(self):
        self.nodes_generated_: int = 0
        self.nodes_expanded_: int = 0
        self.runtime_: float = 0
        self.start_time_: float = 0
        self.solution_: list = None
        self.start_ = None
        self.goal_ = None
