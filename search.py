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

class Vertex:
    def __init__(self, n):
        self.name = n
        self.neighbors = util.Stack()
        self.parent = 0
        self.action = 'Stop'
        self.actionlist = list()


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

    print "Start:", problem.getStartState()
    print "Is the start a goal?", problem.isGoalState(problem.getStartState())
    print "Start's successors:", problem.getSuccessors(problem.getStartState())
    """
    "*** YOUR CODE HERE ***"

    # init
    stack = util.Stack()
    pacman_state = problem.getStartState()
    prev_state = pacman_state
    explored = list()
    action = list()

    # create graph and search
    while not problem.isGoalState(pacman_state):
        # explore
        if(pacman_state not in explored):
            # create vertex
            vertex = Vertex(pacman_state)
            vertex.parent = prev_state
            prev_state = pacman_state

            # add neighbors
            for node in problem.getSuccessors(vertex.name):
                if(node[0] != vertex.parent):
                    vertex.neighbors.push(node)

            # mark expolred
            explored.append(pacman_state)

            # choose next vertex to explored
            # jump back if no neighbors
            if(vertex.neighbors.isEmpty()):
                pacman_state = vertex.parent
            # pop a neighbor
            else:
                node = vertex.neighbors.pop()
                pacman_state = node[0]
                vertex.action = node[1]
                stack.push(vertex)

        # jump back
        else:
            # get parent
            vertex = stack.pop()
            # jump back if no neighbor
            if(vertex.neighbors.isEmpty()):
                pacman_state = vertex.parent
            # pop a neighbor
            else:
                node = vertex.neighbors.pop()
                pacman_state = node[0]
                vertex.action = node[1]
                stack.push(vertex)

    while(not stack.isEmpty()):
        action.append(stack.pop().action)
    action.reverse()
    return action


def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"

    # init
    q_next = util.Queue()
    q_action = util.Queue()
    q_vertices = util.Stack()

    pacman_state = problem.getStartState()
    parent = pacman_state
    action = 'Stop'

    explored = list()
    action_list = list()

    q_next.push((pacman_state, parent, action))

    # create graph and search
    while not problem.isGoalState(pacman_state):
        # choose next vertex to explored
        if(not q_next.isEmpty()):
            move = q_next.pop()
            pacman_state = move[0]
            parent = move[1]
            action = move[2]

        # explore
        if(pacman_state not in explored):
            # create vertex
            vertex = Vertex(pacman_state)
            vertex.parent = parent
            vertex.action = action

            # add children
            if(not problem.isGoalState(pacman_state)):
                for node in problem.getSuccessors(vertex.name):
                    if(node[0] != vertex.parent and node[0] not in explored):
                        q_next.push((node[0], vertex.name, node[1]))

            # mark expolred
            explored.append(pacman_state)
            q_vertices.push(vertex)

    # traceback
    vertex = q_vertices.pop()
    action_list.append(vertex.action)
    parent = vertex.parent
    while(not q_vertices.isEmpty()):
        vertex = q_vertices.pop()
        if(vertex.name == parent and vertex.name != problem.getStartState()):
            parent = vertex.parent
            action_list.append(vertex.action)

    action_list.reverse()
    return action_list

def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
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
    util.raiseNotDefined()


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
