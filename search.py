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
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print("Start:", problem.getStartState())
    print("Is the start a goal?", problem.isGoalState(problem.getStartState()))
    print("Start's successors:", problem.getSuccessors(problem.getStartState()))
    """
    "*** YOUR CODE HERE ***"

    ##
    # Variable Declarations:
    # returnList: A list of actions that will lead the agent from the start to the goal
    # route: A variable to store the route traversed by pacman from the startState to the goalState using the DFS algorithm
    # frontier: A LIFO implemented stack consisting of explored states
    # exploredSet: A set of already explored states, kept for the purpose of avoiding re-exploring already expanded states (will be declared later)
    ##
    returnList = []
    route = []
    frontier = util.Stack()

    # 1) Initialize frontier using the initial state of the problem
    # Note: Even though a the getSuccessors function returns a list of triples (successor, action, stepCost), the implementation of the LIFO stack in util
    # only allows for one item to be pushed onto the stack. Since only the 'action' is relevant in this search method, we will drop the 'stepCost' in order to 
    # represent each next state. Thus, each state here will be represented as a double (coordinates, action).
    startState = (problem.getStartState(), route)
    frontier.push(startState)

    # 2) Initialize explored set to be empty
    exploredSet = []
    
    # 3) Loop through frontier as long as it is not empty
    while frontier.isEmpty() == False:
        # a) Choose a leaf node and remove it from the frontier
        (state, route) = frontier.pop()
        # b) If the node contains a goal state then return the corresponding solution
        if problem.isGoalState(state):
            returnList = route
            return returnList
        # c) Add the node to the explored set
        if state not in exploredSet:
            exploredSet.append(state)
            # d) For each successor, extract it's coordinates and direction and use them to create the next state to add to the fringe
            for x in problem.getSuccessors(state):
                successorCoordinates = x[0]
                successorRoute = [x[1]]
                modifiedRoute = route + successorRoute
                nextState = (successorCoordinates, modifiedRoute)
                frontier.push(nextState)
    
    return returnList

    util.raiseNotDefined()

def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"
    ##
    # Variable Declarations:
    # returnList: A list of actions that will lead the agent from the start to the goal
    # route: A variable to store the route traversed by pacman from the startState to the goalState using the BFS algorithm
    # frontier: A container with a FIFO queueing policy consisting of explored states
    # exploredSet: A set of already explored states, kept for the purpose of avoiding re-exploring already expanded states (will be declared later)
    ##
    returnList = []
    route = []
    frontier = util.Queue()
    
    # 1) Initialize frontier using the initial state of the problem
    startNode = (problem.getStartState(), route)
    frontier.push(startNode)
    
    # 2) Initialize explored set to be empty
    exploredSet = []

    # 3) Loop through frontier as long as it is not empty
    while frontier.isEmpty() == False:
        # a) Choose a leaf node and remove it from the frontier
        (state, route) = frontier.pop()
        # b) If the node contains a goal state then return the corresponding solution
        if problem.isGoalState(state):
            returnList = route
            return returnList
        # c) Add the node to the explored set
        if state not in exploredSet:
            exploredSet.append(state)
            # d) For each successor, extract it's coordinates and direction and use them to create the next state to add to the fringe
            for x in problem.getSuccessors(state):
                successorCoordinates = x[0]
                successorRoute = [x[1]]
                modifiedRoute = route + successorRoute
                nextState = (successorCoordinates, modifiedRoute)
                frontier.push(nextState)
    
    return returnList

    util.raiseNotDefined()

def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    ##
    # Variable Declarations:
    # returnList: A list of actions that will lead the agent from the start to the goal
    # route: A variable to store the route traversed by pacman from the startState to the goalState using the UCS algorithm
    # frontier: A container with a priority-queue queueing policy consisting of explored states
    # exploredSet: A set of already explored states, kept for the purpose of avoiding re-exploring already expanded states (will be declared later)
    # cost: A variable to record the cost of each successor action
    ##
    returnList = []
    route = []
    frontier = util.PriorityQueue()
    cost = 0
    
    # 1) Initialize frontier using the initial state of the problem
    startNode = (problem.getStartState(), route, cost)
    frontier.push(startNode, cost)
    
    # 2) Initialize explored set to be empty
    exploredSet = []

    # 3) Loop through frontier as long as it is not empty
    while frontier.isEmpty() == False:
        # a) Choose a leaf node and remove it from the frontier
        (state, route, cost) = frontier.pop()
        # b) If the node contains a goal state then return the corresponding solution
        if problem.isGoalState(state):
            returnList = route
            return returnList
        # c) Add the node to the explored set
        if state not in exploredSet:
            exploredSet.append(state)
             # d) For each successor, extract it's coordinates, direction and cost. Use them to create the next state to add to the fringe
            for x in problem.getSuccessors(state):
                nextState = (x[0], route + [x[1]], cost + x[2])
                frontier.push(nextState, cost + x[2])
    return returnList
    util.raiseNotDefined()

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"
    ##
    # Variable Declarations:
    # returnList: A list of actions that will lead the agent from the start to the goal
    # route: A variable to store the route traversed by pacman from the startState to the goalState using the A* Search algorithm
    # frontier: A container with a priority-queue queueing policy consisting of explored states
    # exploredSet: A set of already explored states, kept for the purpose of avoiding re-exploring already expanded states (will be declared later)
    # cost: A variable to record the cost f(n) of each successor action, defined by f(n) = g(n) + h(n) where g(n) gives the cost of the path from
    # the start node to the node n, and h(n) is the estimated cost of the cheapest path from node n to the goal using the Manhattan distance heuristic.
    ##
    returnList = []
    route = []
    frontier = util.PriorityQueue()
    cost = 0
    
    # 1) Initialize frontier using the initial state of the problem
    startNode = (problem.getStartState(), route, cost)
    frontier.push(startNode, cost)
    
    # 2) Initialize explored set to be empty
    exploredSet = []

    # 3) Loop through frontier as long as it is not empty
    while frontier.isEmpty() == False:
        # a) Choose a leaf node and remove it from the frontier
        (state, route, cost) = frontier.pop()
        # b) If the node contains a goal state then return the corresponding solution
        if problem.isGoalState(state):
            returnList = route
            return returnList
        # c) Add the node to the explored set
        if state not in exploredSet:
            exploredSet.append(state)
             # d) For each successor, extract it's coordinates, direction and cost (f(n) + g(n)). Use them to create the next state to add to the fringe
            for x in problem.getSuccessors(state):
                successorRoute = [x[1]]
                gn = cost + x[2] # g(n) gives the cost of the path from the start node to the node n
                hn = heuristic(x[0], problem) # h(n) gives the estimated cost of the cheapest path from node n to the goal 
                nextState = (x[0], route + [x[1]], gn)
                frontier.push(nextState, (gn + hn))
    return returnList
    util.raiseNotDefined()


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
