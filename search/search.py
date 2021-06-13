# search.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC Berkeley, including a link to http://ai.berkeley.edu.
# 
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and
# Pieter Abbeel (pabbeel@cs.berkeley.edu).


"""
In search.py, you will implement generic search algorithms which are called by
Pacman agents (in searchAgents.py).
"""

import util


class SearchProblem:
    """
    This class outlines the structure of a search problem, but doesn't implement
    any of the methods (in object-oriented terminology: an abstract class).

    You do not need to change anything in this class, ever.
    """

    def getStartState(self):
        """
        Returns the start state for the search problem.
        """
        util.raiseNotDefined()

    def isGoalState(self, state):
        """
          state: Search state

        Returns True if and only if the state is a valid goal state.
        """
        util.raiseNotDefined()

    def getSuccessors(self, state):
        """
          state: Search state

        For a given state, this should return a list of triples, (successor,
        action, stepCost), where 'successor' is a successor to the current
        state, 'action' is the action required to get there, and 'stepCost' is
        the incremental cost of expanding to that successor.
        """
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        """
         actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.
        The sequence must be composed of legal moves.
        """
        util.raiseNotDefined()

# This generic search can vary in terms of the structure used - the fringe
def generic_search(structure, problem):
    visited = {}
    solution = []
    path = {}
    goal = False

    start = problem.getStartState()
    # We need this annotation
    structure.push((start,[]))

    # If the current position is the goal, exit search fn
    if problem.isGoalState(start):
        return solution

    while not (structure.isEmpty()):
        node = structure.pop()
        
        ##print("Visiting ", node)
        # We found the given goal, break while
        if problem.isGoalState(node[0]):
            return node[1]
        
        # Here we create the given path for each sucessor for the given state
        if node[0] not in visited.keys():
            for suc in problem.getSuccessors(node[0]):
                if suc[0] not in visited.keys():
                    path[suc[0]] = node[0]
                    structure.push((suc[0], node[1]+[suc[1]]))
        
        visited[node[0]] = node[1]
    
    solution = node[1]
    print("SOLUTION: "+str(len(solution))+"\n");
    return solution

def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other maze, the
    sequence of moves will be incorrect, so only use this for tinyMaze.
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return  [s, s, w, s, w, w, s, w]

def depthFirstSearch(problem):
    """
    Depth-First Search that returns a list of actions that reaches 
    goal. 
    """
    print("Start:", problem.getStartState())
    # print("Is the start a goal?", problem.isGoalState(problem.getStartState()))
    print("Start's successors:", problem.getSuccessors(problem.getStartState()))
    print()

    struct = util.Stack()
    return generic_search(struct, problem)

# Same implementation as DFS, except that we search horizontally
def breadthFirstSearch(problem):
    """
    Breadth-First Search that returns a list of actions that reaches the goal
    goal. 
    """
    struct = util.Queue()
    return generic_search(struct, problem)
    

def uniformCostSearch(problem):
    """
    Uniform-Cost Search that returns a list of actions that reaches the
    goal. 
    """
    visited = {}
    structure = util.PriorityQueue();
    start = problem.getStartState()
    structure.push((start,[]) ,0)

    while not (structure.isEmpty()):
        node = structure.pop()
        
        # We found the given goal, break while
        if problem.isGoalState(node[0]):
            break
        
        if node[0] not in visited.keys():
            for suc in problem.getSuccessors(node[0]):
                if suc[0] not in visited.keys():
                    pathCost = node[1] + [suc[1]]
                    structure.push((suc[0], pathCost), problem.getCostOfActions(pathCost))   # defined cost of path

        visited[node[0]] = node[1]
    
    solution = node[1] # actions to goal
    return solution


def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    visited = {}
    structure = util.PriorityQueue();
    start = problem.getStartState()
    structure.push((start,[]), nullHeuristic(start, problem))
    pathCost = 0

    # If the current position is the goal, exit 
    if problem.isGoalState(start):
        return []

    while not (structure.isEmpty()):
        node = structure.pop()
        # We found the given goal, break while
        if problem.isGoalState(node[0]):
            break
        
        if node[0] not in visited.keys():
            for suc in problem.getSuccessors(node[0]):
                if suc[0] not in visited.keys():
                    path = node[1] + [suc[1]]
                    pathCost = problem.getCostOfActions(path) + heuristic(suc[0], problem)
                    structure.push((suc[0], path), pathCost)   # defined cost of path
        visited[node[0]] = node[1]

    solution = node[1] # actions to goal
    return solution


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
