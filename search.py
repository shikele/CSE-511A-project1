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
    process_stack = util.Stack()
    #action_stack = util.Stack()
    
    node_visited = []
    action_taken = []
    node_processed = []
    last_branching_node_arr = []
    #path = []
    #counter = 0
    #get start node and push it onto the stack
    start_node = problem.getStartState()
    process_stack.push(start_node)
    
    print "Is the start a goal?", problem.isGoalState(problem.getStartState())
    end_node = False
    #if the successor fringe not is not the goal state, keep searching
    while not process_stack.isEmpty():
        
        current_node = process_stack.pop()
        if len(current_node) == 2:
            current_node = (current_node,'')
        #print(current_node)
        if current_node[0] in node_visited or current_node in node_processed:
            #del node_visited[-1]
            #del action_taken[-1]
            #if end_node:
            #    node_processed.append(current_node[0])
                

            continue
        if problem.isGoalState(current_node[0]):
            #print(len(process_stack))
            #print(len(action_stack))
            print("success")
            print len(node_visited)
            print len(action_taken)
            action_taken = action_taken[1:]
            return action_taken
        #if not end_node:
        
            

        ##This part will take care of the end note situation
        if end_node: 
            print(current_node[0])
            
            last_branch_node = last_branching_node_arr.pop()
            distance = abs(last_branch_node[0]-current_node[0][0])+abs(last_branch_node[1]-current_node[0][1])
            #search
            while distance >1:
                last_branch_node = last_branching_node_arr.pop()
                distance = abs(last_branch_node[0]-current_node[0][0])+abs(last_branch_node[1]-current_node[0][1])
            #print "Here*****************************************"
            print last_branch_node

            node_index = node_visited.index(last_branch_node)
            #print node_visited[:node_index+1]
            node_visited = node_visited[:node_index+1]
            action_taken = action_taken[:node_index+1]
            #print node_visited
            node_processed.extend(node_visited[node_index:])
            end_node = False

        
        node_visited.append(current_node[0])
        action_taken.append(current_node[1])



            
        #print "current node appened", current_node[0]
        #print node_visited
        
        #print action_taken

        
        
        
        successors = problem.getSuccessors(current_node[0])
        target = len(successors)
        for side_node in successors:
            if side_node[0] in node_visited or side_node[0] in node_processed:
                target -=1

        if target == 0 :
            print "This is end node*******************************"
            end_node = True
        elif target ==2:
            last_branching_node_arr.append(current_node[0])
        elif target == 3:
            last_branching_node_arr.append(current_node[0])
            last_branching_node_arr.append(current_node[0])
            
            
          
            
                
        for fringe_node in successors:
            
                #counter += 1
            #push the visited node to the stack
            
            process_stack.push(fringe_node)
            #action_stack.push(fringe_node[1])
            
          
            print "fringe node is:" ,fringe_node

            
        
        print("here wait a sec")
            
    print("Fail")
            
    
    
    
    
    
    #util.raiseNotDefined()

def breadthFirstSearch(problem):
    """
    Search the shallowest nodes in the search tree first.
    [2nd Edition: p 73, 3rd Edition: p 82]
    """
    "*** YOUR CODE HERE ***"
    util.raiseNotDefined()

def uniformCostSearch(problem):
    "Search the node of least total cost first. "
    "*** YOUR CODE HERE ***"
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
    util.raiseNotDefined()


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
