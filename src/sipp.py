# Classes to implement Safe Interval Path Planning for the Flatland domain
# contains SIPP search nodes, SIPP searcher and methods to find safe intervals for given existing paths

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

class sipp_node:
    """
    Defines a Safe Interval Path Planning node with time and interval instance variables.
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
        self.interval_: tuple = (0, float('inf'))
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
        return str((self.state_, self.interval_, self.action_.move_, self.t))

        #return str((self.state_, self.t))

    def __repr__(self):
        return self.state_.__repr__()

        #return (self.state_, self.t).__repr__()

    def __eq__(self, other):
        
        if (other == None):
            return False
        #return self.state_ == other.state_ and self.t == other.t and self.action_.move_ == other.action_.move_ 
        return self.state_ == other.state_ and self.interval_[1] == other.interval_[1] and self.action_.move_ == other.action_.move_ 
        #return self.state_ == other.state_ 
        #return hash((self.state_, self.t)) == hash((other.state_, other.t))

    def __hash__(self):
        #return hash((self.state_)) 
        return hash((self.state_, self.interval_[1], self.action_.move_)) 
        #return hash(self.state_) + hash( self.interval_) + hash(self.action_.move_)

        #return hash((self.state_, self.t, self.action_.move_)) 

class SIPP():
    """
    Defines the Safe Interval Path Planning algorithm
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

        global safe_intervals_all

        if safe_intervals_all is None or self.reset:
            safe_intervals_all = grid_interval_table(self.rail.height, self.rail.width)

    def get_path(self, start: tuple, start_direction: int, goal: tuple, agent_id: int, existing_paths: list, max_timestep: int, start_collision_check = False):

        global safe_intervals_all
        # initialise lists and dictionaries
        path = []
        #all_nodes_list_ = {}
        #nodes_expanded_ = 0
        #nodes_generated_ = 0
        self.status_ = "Failed"
        self.open_list_.clear()
        self.all_nodes_list_.clear()
        self.reset_statistic()

        action = graph_expander.graph_action(start_direction, 0)

        # generate start node
        start_node: sipp_node = self.generate(start, action, parent=None, goal_=goal, t=0)

        # Our state will be a list consisting of the location and the direction.
        self.open_list_.push(start_node)
        self.all_nodes_list_[start_node] = start_node

        # compute all safe_intervals ahead of time and store in a dictionary
        import time

        global safe_intervals_all

        start = time.time()
        #self.safe_intervals_all = {(x, y): self.determine_safe_intervals((x,y), existing_paths=existing_paths) for x in range(self.rail.height) for y in range(self.rail.width) }

        try:
            latest_path = existing_paths[-1]
        except:
            latest_path = []
        for x in range(self.rail.height):
            for y in range(self.rail.width):
                safe_intervals_all.update((x,y), latest_path)
        end = time.time()

        #print(f'safe intervals counted in {end - start}')

        # continue while there are still nods on OPEN
        while (len(self.open_list_) > 0):
            
            current: sipp_node = self.open_list_.pop()
            current.close()
            self.nodes_expanded_ +=1

            # goal example. if successful, return the solution
            if self.is_goal(current.state_, goal):
                path = self.solution_time(current)
                self.status_ = "Success"
                break

            # get successors from the current node
            successors = self.get_successors(current, existing_paths, goal)

            # loop through all successors
            for s in successors:

                # if s has not been visited yet, set g and f to infinity
                if s not in self.all_nodes_list_:
                    s.g_ = float('inf')
                    s.f_ = float('inf')
                
                cost = s.t - current.t
                # if s has been visited, this should essentially eval to false
                if s.g_ > current.g_ + cost:
                    s.g_ = current.g_ + cost
                    s.f_ = s.g_ + s.h_
                    s.open_handle_ = self.open_list_.push(s)
                    self.all_nodes_list_[s] = s
                    self.nodes_generated_+= 1

                    # succ_node only have the same hash and state comparing with the on in the all nodes list
                    # It's not the one in the all nodes list,  we need the real node in the all nodes list.
                    exist = self.all_nodes_list_[s]
                    if not exist.is_closed():
                        self.relax(exist, s)
        
        # as SIPP nodes have a time component, we repeat states where an agent "waits"
        # loop through the path if it exists and repeat nodes for time differences between them
        if len(path) >0:
            redo_path = []
            for i in range(len(path)):
                if i>0:
                    for _ in range(path[i].t - path[i-1].t):
                        redo_path.append(path[i-1].state_)

                elif i == len(path):
                    redo_path.append(path[i].state_)
            
            # append the goal
            redo_path.append(goal)
            path = redo_path    

        # check for collisions through start positions
        # if there is a collision, return an empty list for that agent
        if start_collision_check:
            start_location_path = path[0]
            last_t_at_start = self.start_position_last_t(path)+1
            for t in range(last_t_at_start):
                if self.reservation_table.is_reserved(start_location_path, t, self.agent_id):
                    return []

        #print(f"{self.agent_id}: generated: {self.nodes_generated_}, expanded:{self.nodes_expanded_}")
                
        return path                 

    def get_successors(self, current, existing_paths, goal):
        """
        Define the successor function
        """
        successors = []
        loc_state = current.state_
        direction_state = current.action_.move_
        valid_transitions = self.rail.get_transitions(loc_state[0],loc_state[1],direction_state)

        for i in range(0,len(valid_transitions)):
                
            # if valid, process the transition
            if valid_transitions[i]:
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

                # generate safe intervals
                #safe_intervals = self.safe_intervals_all[(new_x, new_y)]
                safe_intervals = safe_intervals_all.get_safe_interval((new_x, new_y))
                m_time = 1 # motion time (it's always 1 as it only takes 1 unit of time to move a train)
                start_t = current.t + m_time
                end_t = current.interval_[1] + m_time  # final time period of corresponding safe interval
                
                # loop through all safe intervals
                for interval in safe_intervals:
                    
                    interval_start, interval_end = interval

                    if interval_start > end_t or interval_end < start_t:
                        continue

                    # earliest arrival time at cfg during the interval with no collisions
                    t = max(start_t, interval_start)

                    # check for collisions
                    conflict = False
                    collision_t = t

                    for p in existing_paths:                        
                        if collision_t < len(p) and p[collision_t] == (new_x,new_y):
                            conflict = True
                        if collision_t < len(p) and p[collision_t] ==(loc_state[0],loc_state[1]) and p[collision_t-1] ==(new_x,new_y):
                            conflict = True
                    if conflict:
                        continue

                    # end guard rails
                    if t > min(end_t, interval[1]):
                        continue

                    loc_updated = (new_x,new_y)
                    direction_updated = action

                    # reuse the graph_action class. the cost is the time to act (t - current timestep)
                    time_to_act = t - current.t #- m_time
                    new_action = graph_expander.graph_action(direction_updated, time_to_act)

                    # generate the successor node
                    succ_node = self.generate(state=loc_updated, action=new_action, parent=current, goal_=goal, t = t, interval=interval)

                    successors.append(succ_node)
        return successors

    def start_position_last_t(self, path):
        # count how long the agent is in the start position for
            start_pos = path[0]

            for i in range(1, len(path)):
                if path[i] == start_pos:
                    continue
                else:
                    break  # stop checking the rest of the list

            return i-1

    def generate(self, state, action, parent: sipp_node, goal_: tuple = None,  t=0, interval = (0, float('inf'))):

        retval = sipp_node()
        retval.state_ = state
        retval.action_ = action
        if (parent == None):
            # initialise the node from scratch
            # NB: we usually do this only for the start node
            retval.g_ = 0
            retval.depth_ = 0
            retval.timestep_ = 0
            retval.interval_ = (0, float('inf'))
            retval.t = 0
        else:
            # initialise the node based on its parent
            retval.g_ = parent.g_ + action.cost_
            retval.depth_ = parent.depth_ + 1
            retval.parent_ = parent
            retval.timestep_= parent.timestep_ + 1
            retval.t = t
            retval.interval_ = interval

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

    def solution_time(self, goal_node: sipp_node):
        # recurse from goal node to start node of the solution
        tmp = goal_node
        sol = []
        while (tmp != None):
            sol.append(tmp)
            tmp = tmp.parent_
            
        sol.reverse()

        return sol

    def relax(self, exist:sipp_node, new:sipp_node):
        if exist.g_ > new.g_:
            exist.f_ = new.f_
            exist.g_ = new.g_
            exist.depth_ = new.depth_
            exist.instance_ = new.instance_
            exist.action_ = new.action_
            exist.timestep_ = new.timestep_
            exist.t = new.t
            exist.interval_ = new.interval_
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

class grid_interval_table:

    width_: int
    height_: int
    table_: list

    def __init__(self,width, height):
        self.width_ = width
        self.height_ = height
        self.table_ = [[None] * int(self.width_) for x in range(int(self.height_))]

    # Check is an location reserved by any other agent
    # @param loc A tuple of (x,y) coordinates.
    # @patam time The timestep.
    # @param agent_id An int of agent_id.
    # @return bool True if reserved.
    def get_safe_interval(self, loc: tuple):
        x = loc[0]
        y = loc[1]
        if self.table_[x][y] is None:
            return [(0,float('inf'))]

        return self.table_[x][y]

    # Add an single reservation to reservation table
    # @param loc A tuple of (x,y) coordinates.
    # @patam time The timestep.
    # @param agent_id An int of agent_id.
    # @return success True if add successful, false if the location reserved by other agent.
    def add_loc(self, loc: tuple, intervals: tuple):
        x = loc[0]
        y = loc[1]
        # if self.table_[x][y] is None:
        #     self.table_[x][y] = {}

        self.table_[x][y] = intervals
        return True

    # Delete a reserve from reservation table
    # @param loc A tuple of (x,y) coordinates.
    # @patam time The timestep.
    # @param agent_id An int of agent_id.
    # @return success True if delete successful, False if reserve doesn't exist
    def del_loc(self, loc: tuple, time: int, intervals: int):
        x = loc[0]
        y = loc[1]
        if self.table_[x][y] is None:
            return False

        if time in self.table_[x][y]:
            if self.table_[x][y][time]== intervals:
                self.table_[x][y][time] = -1
                return True
            else:
                return False
        else:
            return False
        
    def determine_interval_t(self, location, path):
        # loop through existing paths and find timesteps when location intersects with a part of the path
        times = [i for i,x in enumerate(path) if x == location]

        # flatten list, then sort and reverse to get increasing times of t
        #times_flat = [item for sublist in times for item in sublist]
        times.sort()
        times.reverse()

        return times
    
    def update_intervals(self, interval_list, times):
        # Update safe intevals with the latest existing paths
        while len(times) >0:
            loop_t = times.pop()
            for interval in interval_list:
                if loop_t == interval[0]:
                    interval_list.remove(interval)
                    if loop_t+1 <= interval[1]:
                        interval_list.append((loop_t+1, interval[1]))
                elif loop_t == interval[1]:
                    interval_list.remove(interval)
                    if loop_t-1 <= interval[0]:
                        interval_list.append((interval[0],loop_t-1))
                elif bisect.bisect(interval,loop_t) == 1:
                    interval_list.remove(interval)
                    interval_list.append((interval[0], loop_t-1))
                    interval_list.append((loop_t+1, interval[1]))

        interval_list.sort()

        return interval_list
    
    def update(self, loc, path):

        interval_times = self.determine_interval_t(loc, path)
        curr_intervals = self.get_safe_interval(loc)
        updated = self.update_intervals(curr_intervals, interval_times)
        self.add_loc(loc, updated)

    # clear the reservation table
    def clear(self):
        self.table_ = [[None] * int(self.width_) for x in range(int(self.height_))]