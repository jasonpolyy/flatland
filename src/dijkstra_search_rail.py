# Implmements Dijkstra search for the Flatland domain
# Contains classes for Dijkstra search and dijkstra search nodes

from lib_piglet.search.search_node import compare_node_g
from lib_piglet.utils.data_structure import bin_heap
from lib_piglet.expanders import graph_expander
import time

try:
    from flatland.core.transition_map import GridTransitionMap
    from flatland.utils.controller import  Directions
except Exception as e:
    exit(1)

import sys

class dijkstra_search_node:

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
        return str(self.state_, self.action_.move_)

        #return str((self.state_, self.t))

    def __repr__(self):
        #return self.state_.__repr__()

        return (self.state_, self.action_.move_).__repr__()

    def __eq__(self, other):
        
        if (other == None):
            return False
        return self.state_ == other.state_ and self.action_.move_ == other.action_.move_ 

    def __hash__(self):
        return hash((self.state_,self.action_.move_))

class dijkstra_search_rail():
    """
    Implement Dijkstra search for the GridTransitionMap domain.
    """

    def __init__(self, open_list: bin_heap, time_limit: int = sys.maxsize):
        """Init method

        Parameters
        ----------
        open_list : bin_heap
            the open list using compare_node_g 
        time_limit : int, optional
            max time limit (not used), by default sys.maxsize
        """
        self.time_limit_ = time_limit

        # no heuristic function in dijkstra search
        self.heuristic_function_ = None

        self.open_list_ = open_list
        self.all_nodes_list_ = {}
        self.nodes_generated_: int = 0
        self.nodes_expanded_: int = 0
        self.runtime_: float = 0
        self.start_time_: float = 0
        self.solution_: list = None
        self.start_: object = None
        self.goal_: object = None
        self.status_: str = None
        self.heuristic_weight_:float = 1.0
        self.max_depth_ = 0

    # get distance from all state to target_state
    # @param target_state Then target_state of the search
    # @return a dictionary contains each state and distance from this state to target state
    def get_path(self, target_state: tuple, rail: GridTransitionMap = None):

        self.open_list_.clear()
        self.all_nodes_list_.clear()
        self.reset_statistic()
        self.max_depth_ = 0
        self.goal_ = target_state
        self.start_time = time.process_time()

        # get the initial action for the given pivot
        for direction in Directions:

            # get all valid transitions for a given direction
            valid_transitions = rail.get_transitions(target_state[0], target_state[1], direction)
            
            # loop through all possible transitions and check if it is a valid move
            for i in range(len(valid_transitions)):

                # if the move is valid, add it to the open list to be expanded
                # assuming the pivot is a valid location, this ensures we always have a start point to begin dijkstra search
                if valid_transitions[i]:
                    action = graph_expander.graph_action(direction, 1)
                    start_node = self.generate(target_state, action, None)
                    self.open_list_.push(start_node)
                    self.all_nodes_list_[start_node] = start_node

        # continue while there are still nods on OPEN
        while (len(self.open_list_) > 0):

            current: dijkstra_search_node = self.open_list_.pop()
            current.close()
            self.nodes_expanded_ +=1
            if current.depth_ > self.max_depth_:
                self.max_depth_ = current.depth_
            # If have time_limit, break time out search.
            if self.time_limit_ < sys.maxsize:
                self.runtime_ = time.process_time() - self.start_time
                if self.runtime_ > self.time_limit_:
                    self.status_ = "Time out"
                    return None

            if self.nodes_expanded_%100000 == 0:
                print(self.nodes_expanded_)

            # expand the current node
            loc_state = current.state_
            #direction_state = current.action_.move_

            for direction in Directions:

                #print(direction)
                valid_transitions = rail.get_transitions(loc_state[0],loc_state[1],direction)

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
                            
                        loc_updated = (new_x,new_y)
                        direction_updated = action

                        new_action = graph_expander.graph_action(direction_updated, 1)

                        succ_node = self.generate(loc_updated, new_action, current)

                        # generate the successor node
                        #succ_node = generate(state=loc_updated, action=new_action, parent=current, goal_=target_state, heuristic_function_ = manhattan_heuristic, heuristic_weight_=1)
                
                        # succ_node not in any list, add it to open list
                        if succ_node not in self.all_nodes_list_:
                            # we need this open_handle_ to update the node in open list in the future
                            succ_node.open_handle_ = self.open_list_.push(succ_node)
                            self.all_nodes_list_[succ_node] = succ_node
                            self.nodes_generated_+= 1

                        # succ_node only have the same hash and state comparing with the on in the all nodes list
                        # It's not the one in the all nodes list,  we need the real node in the all nodes list.
                        exist = self.all_nodes_list_[succ_node]
                        if not exist.is_closed():
                            self.relax(exist, succ_node)
                
        # OPEN list is exhausted, dijkstra finish
        self.solution_ = self.solution()
        self.status_ = "Success"
        self.runtime_ = time.process_time() - self.start_time
        return self.solution_

    def relax(self, exist:dijkstra_search_node, new:dijkstra_search_node):
        if exist.g_ > new.g_:
            exist.f_ = new.f_
            exist.g_ = new.g_
            exist.h_ = new.h_
            exist.parent_ = new.parent_
            if exist.open_handle_ is not None:
                # If handle exist, we are using bin_heap. We need to tell bin_heap one element's value
                # is decreased. Bin_heap will update the heap to maintain priority structure.
                self.open_list_.decrease(exist.open_handle_)

    # extract the computed solution by following backpointers
    def solution(self):
        sol = {}
        for node in self.all_nodes_list_:
            sol[node.state_] = node.g_

        return sol
    
    def get_statistic(self):
        sta_ = [self.status_]
        if self.solution_!=None:
            sta_ += self.solution_.get_solution_info()
        else:
            sta_ += [None, None]
        sta_ += [self.nodes_expanded_,
                self.nodes_generated_,
                round(self.runtime_,4),
                self.start_,
                self.goal_,
                self.expander_]

        return sta_
    
    def generate(self, state, action, parent: dijkstra_search_node):

        retval = dijkstra_search_node()
        retval.state_ = state
        retval.action_ = action
        if (parent == None):
            # initialise the node from scratch
            # NB: we usually do this only for the start node
            retval.g_ = 0
            retval.depth_ = 0
            retval.timestep_ = 0
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
            retval.h_ = self.heuristic_function_(self.expander_.domain_,retval.state_, self.goal_)
            retval.f_ = retval.g_ + retval.h_ * self.heuristic_weight_
        return retval


    def reset_statistic(self):
        self.nodes_generated_: int = 0
        self.nodes_expanded_: int = 0
        self.runtime_: float = 0
        self.start_time_: float = 0
        self.solution_ = None
        self.start_ = None
        self.goal_ = None


def run_dijkstra(rail: GridTransitionMap, pivot: tuple) -> dict:
    """Run Dijkstra search for the GridTransitionMap domain and return 
    all solutions for a given pivot.

    Parameters
    ----------
    rail : GridTransitionMap
        the domain
    pivot : tuple
        pivot in a differential heuristic

    Returns
    -------
    dict
        returns a dictionary where keys are all valid positions on the rail map
        and values are the cost to reach them from the pivot.
    """
    # initialise the open list as a binary heap comparing cost g
    open = bin_heap(compare_node_g)

    # initialise a dijkstra search with the open list
    searcher = dijkstra_search_rail(open)

    # run dijkstra search
    path = searcher.get_path(pivot, rail)

    # return the solution 
    #sol = searcher.solution()
    return path
