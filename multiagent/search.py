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
        self.cost = 0


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


def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"

    # init
    q_next = util.Queue()

    pacman_state = problem.getStartState()
    explored = list()
    action_list = list()

    q_next.push((pacman_state, action_list))
    goal = problem.isGoalState(pacman_state)

    # create graph and search
    while not goal and not q_next.isEmpty():
        #print goal, q_next.isEmpty()
        # choose next vertex to explored
        if(not q_next.isEmpty()):
            move = q_next.pop()
            pacman_state = move[0]
            action_list = list(move[1])
            goal = problem.isGoalState(pacman_state)
        # explore
        if(pacman_state not in explored):
            # create vertex
            vertex = Vertex(pacman_state)
            vertex.actionlist = action_list

            if(goal): break
            # add children
            for node in problem.getSuccessors(vertex.name):
                if(node[0] not in explored):
                    alist = list(vertex.actionlist)
                    alist.append(node[1])
                    q_next.push((node[0], alist))

            # mark expolred
            explored.append(pacman_state)

    # traceback

    return vertex.actionlist

# Abbreviations
bfs = breadthFirstSearch
