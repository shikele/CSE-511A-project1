# search.py
# ---------
# Licensing Information: Please do not distribute or publish solutions to this
# project. You are free to use and extend these projects for educational
# purposes. The Pacman AI projects were developed at UC Berkeley, primarily by
# John DeNero (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# For more info, see http://inst.eecs.berkeley.edu/~cs188/sp09/pacman.html

"""
In search.py, you will implement generic search algorithms which are called
by Pacman agents (in searchAgents.py).
"""

import util
import random

class SearchProblem:
    """
    This class outlines the structure of a search problem, but doesn't implement
    any of the methods (in object-oriented terminology: an abstract class).

    You do not need to change anything in this class, ever.
    """

    def getStartState(self):
        """
        Returns the start state for the search problem
        """
        util.raiseNotDefined()

    def isGoalState(self, state):
        """
          state: Search state

        Returns True if and only if the state is a valid goal state
        """
        util.raiseNotDefined()

    def getSuccessors(self, state):
        """
          state: Search state

        For a given state, this should return a list of triples,
        (successor, action, stepCost), where 'successor' is a
        successor to the current state, 'action' is the action
        required to get there, and 'stepCost' is the incremental
        cost of expanding to that successor
        """
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        """
         actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.  The sequence must
        be composed of legal moves
        """
        util.raiseNotDefined()


def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other
    maze, the sequence of moves will be incorrect, so only use this for tinyMaze
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return  [s,s,w,s,w,w,s,w]

def depthFirstSearch(problem):
    """
    Search the deepest nodes in the search tree first
    [2nd Edition: p 75, 3rd Edition: p 87]

    Your search algorithm needs to return a list of actions that reaches
    the goal.  Make sure to implement a graph search algorithm
    [2nd Edition: Fig. 3.18, 3rd Edition: Fig 3.7].

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print "Start:", problem.getStartState()
    print "Is the start a goal?", problem.isGoalState(problem.getStartState())
    print "Start's successors:", problem.getSuccessors(problem.getStartState())
    """
    "*** YOUR CODE HERE ***"
    # nodes to be processed, format: (location, action, action cost)
    process_stack = util.Stack()
    
    # nodes visited, store location -(x,y)
    node_visited = []
    # store action - 'South'/'West'/'North'/'East'
    action_taken = []
    # nodes have been visited but not on the return path, store location -(x,y)
    node_processed = []
    # nodes still have 2 or more successors that haven't been visited
    last_branching_node_arr = []


    #get start node and push it onto the stack
    start_node = problem.getStartState()
    process_stack.push(start_node)
    
    #print "Is the start a goal?", problem.isGoalState(problem.getStartState())
    end_node = False
    #if the successor fringe node is not the goal state, keep searching
    while not process_stack.isEmpty():
        
        current_node = process_stack.pop()
        # add direction to start node
        if len(current_node) == 2:
            current_node = (current_node,'')
        if current_node[0] in node_visited or current_node in node_processed:
            continue
        if problem.isGoalState(current_node[0]):
            print("success")
            #print len(node_visited)
            #print len(action_taken)
            action_taken.append(current_node[1])
            # exclude action_taken[0] for the start node
            action_taken = action_taken[1:]
            return action_taken


        ##This part will take care of the end note situation
        if end_node: 
            #print(current_node[0])
            
            last_branch_node = last_branching_node_arr.pop()
            distance = abs(last_branch_node[0]-current_node[0][0])+abs(last_branch_node[1]-current_node[0][1])
            #search
            while distance >1:
                last_branch_node = last_branching_node_arr.pop()
                distance = abs(last_branch_node[0]-current_node[0][0])+abs(last_branch_node[1]-current_node[0][1])
            #print last_branch_node

            node_index = node_visited.index(last_branch_node)
            node_processed.extend(node_visited[node_index+1:])
            node_visited = node_visited[:node_index+1]
            action_taken = action_taken[:node_index+1]
            
            end_node = False

        
        node_visited.append(current_node[0])
        action_taken.append(current_node[1])

        
        successors = problem.getSuccessors(current_node[0])
        target = len(successors)
        for side_node in successors:
            if side_node[0] in node_visited or side_node[0] in node_processed:
                target -=1

        if target == 0 :
            #print "This is end node*******************************"
            end_node = True
        # add last_branching_node for (successors haven't been visited -1) to retreat
        elif target ==2:
            last_branching_node_arr.append(current_node[0])
        elif target == 3:
            last_branching_node_arr.append(current_node[0])
            last_branching_node_arr.append(current_node[0])

                
        for fringe_node in successors:
            #push the node to the to-visit stack
            process_stack.push(fringe_node)
          
            #print "fringe node is:" ,fringe_node
        
        #print("here wait a sec")
            
    print("Fail")
    return action_taken[1:]
    util.raiseNotDefined()



def breadthFirstSearch(problem):
    """
    Search the shallowest nodes in the search tree first.
    [2nd Edition: p 73, 3rd Edition: p 82]
    """
    "*** YOUR CODE HERE ***"
    # nodes to be processed, format: (location, action, action cost)
    process_queue = util.Queue()

    # nodes visited, store location -(x,y)
    node_visited = []

    # get start node and push it onto the stack
    start_node = problem.getStartState()
    # create a list[] for each path
    process_queue.push((start_node,[]))

    # if the successor fringe node is not the goal state, keep searching
    while not process_queue.isEmpty():

        current_node = process_queue.pop()
        if current_node[0] in node_visited:
            continue
        if problem.isGoalState(current_node[0]):
            print("success")
            # action is the path saved in current node
            return current_node[1]

        node_visited.append(current_node[0])
        successors = problem.getSuccessors(current_node[0])
        # restore the path from the start to current node
        parent_path = current_node[1]
        for fringe_node in successors:
            location = fringe_node[0]
            # connect the path from current node to each fringe node
            fringe_path = parent_path + [fringe_node[1]]
            # push the node to the to-visit stack
            process_queue.push((location, fringe_path))

    print("Fail")
    util.raiseNotDefined()



def uniformCostSearch(problem):
    "Search the node of least total cost first. "
    "*** YOUR CODE HERE ***"
    # nodes to be processed, format: (location, action, action cost)
    process_queue = util.PriorityQueue()

    # nodes visited, store location -(x,y)
    node_visited = []

    # get start node and push it onto the stack
    start_node = problem.getStartState()
    # create a list[] for each path
    process_queue.push((start_node, []),0)

    # if the successor fringe node is not the goal state, keep searching
    while not process_queue.isEmpty():

        current_node = process_queue.pop()
        if current_node[0] in node_visited:
            continue
        if problem.isGoalState(current_node[0]):
            print("success")
            # action is the path saved in current node
            return current_node[1]

        node_visited.append(current_node[0])
        successors = problem.getSuccessors(current_node[0])
        # restore the path from the start to current node
        parent_path = current_node[1]
        for fringe_node in successors:
            location = fringe_node[0]
            # connect the path from current node to each fringe node
            fringe_path = parent_path + [fringe_node[1]]
            # push the node to the to-visit stack
            process_queue.push((location, fringe_path),fringe_node[2])

    print("Fail")
    util.raiseNotDefined()

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    "Search the node that has the lowest combined cost and heuristic first."
    "*** YOUR CODE HERE ***"
    # nodes to be processed, format: (location, action, action cost)
    process_queue = util.PriorityQueue()

    # nodes visited, store location -(x,y)
    node_visited = []

    # get start node and push it onto the stack
    start_node = problem.getStartState()
    # create a list[] for each path
    process_queue.push((start_node, []), 0)

    # if the successor fringe node is not the goal state, keep searching
    while not process_queue.isEmpty():

        current_node = process_queue.pop()
        if current_node[0] in node_visited:
            continue
        if problem.isGoalState(current_node[0]):
            print("success")
            # action is the path saved in current node
            return current_node[1]

        node_visited.append(current_node[0])
        successors = problem.getSuccessors(current_node[0])
        # restore the path from the start to current node
        parent_path = current_node[1]
        for fringe_node in successors:
            location = fringe_node[0]
            # connect the path from current node to each fringe node
            fringe_path = parent_path + [fringe_node[1]]
            # push the node to the to-visit stack
            process_queue.push((location, fringe_path), fringe_node[2])

    print("Fail")
    util.raiseNotDefined()


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
