# multiAgents.py
# --------------
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


from util import manhattanDistance
from game import Directions
import random, util
import search
import searchAgents
from game import Agent

class ReflexAgent(Agent):
    """
      A reflex agent chooses an action at each choice point by examining
      its alternatives via a state evaluation function.

      The code below is provided as a guide.  You are welcome to change
      it in any way you see fit, so long as you don't touch our method
      headers.
    """


    def getAction(self, gameState):
        """
        You do not need to change this method, but you're welcome to.

        getAction chooses among the best options according to the evaluation function.

        Just like in the previous project, getAction takes a GameState and returns
        some Directions.X for some X in the set {North, South, West, East, Stop}
        """
        # Collect legal moves and successor states
        legalMoves = gameState.getLegalActions()

        # Choose one of the best actions
        scores = [self.evaluationFunction(gameState, action) for action in legalMoves]
        bestScore = max(scores)
        bestIndices = [index for index in range(len(scores)) if scores[index] == bestScore]
        chosenIndex = random.choice(bestIndices) # Pick randomly among the best

        "Add more of your code here if you want to"

        return legalMoves[chosenIndex]

    def evaluationFunction(self, currentGameState, action):
        """
        Design a better evaluation function here.

        The evaluation function takes in the current and proposed successor
        GameStates (pacman.py) and returns a number, where higher numbers are better.

        The code below extracts some useful information from the state, like the
        remaining food (newFood) and Pacman position after moving (newPos).
        newScaredTimes holds the number of moves that each ghost will remain
        scared because of Pacman having eaten a power pellet.

        Print out these variables to see what you're getting, then combine them
        to create a masterful evaluation function.
        """
        # Useful information you can extract from a GameState (pacman.py)
        successorGameState = currentGameState.generatePacmanSuccessor(action)
        newPos = successorGameState.getPacmanPosition()
        newFood = successorGameState.getFood()
        newGhostStates = successorGameState.getGhostStates()
        newScaredTimes = [ghostState.scaredTimer for ghostState in newGhostStates]

        "*** YOUR CODE HERE ***"

        # food points
        f_points = 1
        food_lst = newFood.asList()
        if food_lst:
            f_dis = min([util.manhattanDistance(newPos, food_pos) for food_pos in food_lst])
            if not f_dis: f_dis = 1
            f_points = f_points * (1.0 / f_dis)

        # capsule points
        c_points = 1.4
        capsules = successorGameState.getCapsules();
        if capsules:
            c_dis = min([util.manhattanDistance(newPos, capsule) for capsule in capsules])
            if not c_dis: c_dis = 1
            c_points = c_points * (1.0 / c_dis)

        # ghost points and buster points
        g_points = -2
        b_points = 0
        ghost_lst = successorGameState.getGhostPositions()
        if ghost_lst:
            g_dis = min([[util.manhattanDistance(newPos, ghost), ghost_index] for ghost_index, ghost in enumerate(ghost_lst)])
            if not g_dis[0]: g_dis[0] = 1
            g_points = g_points * (1.0 / g_dis[0])
            if g_dis[0] < 2: g_points = -20.0 / g_dis[0]
            if newScaredTimes[g_dis[1]]:
                g_points = 0
                b_points = 4.0 * (1.0 / g_dis[0]) * (newScaredTimes[g_dis[1]] / 40.0)

        return successorGameState.getScore() + f_points + g_points + c_points + b_points

def scoreEvaluationFunction(currentGameState):
    """
      This default evaluation function just returns the score of the state.
      The score is the same one displayed in the Pacman GUI.

      This evaluation function is meant for use with adversarial search agents
      (not reflex agents).
    """
    return currentGameState.getScore()

class MultiAgentSearchAgent(Agent):
    """
      This class provides some common elements to all of your
      multi-agent searchers.  Any methods defined here will be available
      to the MinimaxPacmanAgent, AlphaBetaPacmanAgent & ExpectimaxPacmanAgent.

      You *do not* need to make any changes here, but you can if you want to
      add functionality to all your adversarial search agents.  Please do not
      remove anything, however.

      Note: this is an abstract class: one that should not be instantiated.  It's
      only partially specified, and designed to be extended.  Agent (game.py)
      is another abstract class.
    """

    def __init__(self, evalFn = 'scoreEvaluationFunction', depth = '2'):
        self.index = 0 # Pacman is always agent index 0
        self.evaluationFunction = util.lookup(evalFn, globals())
        self.depth = int(depth)

class MinimaxAgent(MultiAgentSearchAgent):
    """
      Your minimax agent (question 2)
    """

    def getAction(self, gameState):
        """
          Returns the minimax action from the current gameState using self.depth
          and self.evaluationFunction.

          Here are some method calls that might be useful when implementing minimax.

          gameState.getLegalActions(agentIndex):
            Returns a list of legal actions for an agent
            agentIndex=0 means Pacman, ghosts are >= 1

          gameState.generateSuccessor(agentIndex, action):
            Returns the successor game state after an agent takes an action

          gameState.getNumAgents():
            Returns the total number of agents in the game
        """
        "*** YOUR CODE HERE ***"

        def value(gameState, agent_index, current_depth):
            if agent_index == gameState.getNumAgents():
                agent_index = 0;
            # Choose Min or Max
            if agent_index == 0:
                return maxValue(gameState, current_depth + 1)[0]
            else:
                return minValue(gameState, agent_index, current_depth)

        def maxValue(gameState, current_depth):
            v_max = (-99999, 'Stop')
            pacman_actions = gameState.getLegalActions(0)
            # leaves
            if not pacman_actions or current_depth > self.depth:
                return (self.evaluationFunction(gameState), 'Stop')
            # Branches
            for pacman_action in pacman_actions:
                newGameState = gameState.generateSuccessor(0, pacman_action)
                v_max = max(v_max, (value(newGameState, 1, current_depth), pacman_action))
            return v_max

        def minValue(gameState, ghost_index, current_depth):
            v_min = 99999
            ghost_actions = gameState.getLegalActions(ghost_index)
            # leaves
            if not ghost_actions:
                return self.evaluationFunction(gameState)
            # Branches
            for ghost_action in ghost_actions:
                newGameState = gameState.generateSuccessor(ghost_index, ghost_action)
                v_min = min(v_min, value(newGameState, ghost_index + 1, current_depth))
            return v_min

        return maxValue(gameState, 1)[1]


class AlphaBetaAgent(MultiAgentSearchAgent):
    """
      Your minimax agent with alpha-beta pruning (question 3)
    """

    def getAction(self, gameState):
        """
          Returns the minimax action using self.depth and self.evaluationFunction
        """
        "*** YOUR CODE HERE ***"

        def value(gameState, agent_index, current_depth, alpha, beta):
            if agent_index == gameState.getNumAgents():
                agent_index = 0;
            # Choose Min or Max
            if agent_index == 0:
                return maxValue(gameState, current_depth + 1, alpha, beta)[0]
            else:
                return minValue(gameState, agent_index, current_depth, alpha, beta)

        def maxValue(gameState, current_depth, alpha, beta):
            v_max = (-99999, 'Stop')
            pacman_actions = gameState.getLegalActions(0)
            # leaves
            if not pacman_actions or current_depth > self.depth:
                return (self.evaluationFunction(gameState), 'Stop')
            # Branches
            for pacman_action in pacman_actions:
                newGameState = gameState.generateSuccessor(0, pacman_action)
                v_max = max(v_max, (value(newGameState, 1, current_depth, alpha, beta), pacman_action))
                if v_max[0] > beta: return v_max
                alpha = max(alpha, v_max[0])
            return v_max

        def minValue(gameState, ghost_index, current_depth, alpha, beta):
            v_min = 99999
            ghost_actions = gameState.getLegalActions(ghost_index)
            # leaves
            if not ghost_actions:
                return self.evaluationFunction(gameState)
            # Branches
            for ghost_action in ghost_actions:
                newGameState = gameState.generateSuccessor(ghost_index, ghost_action)
                v_min = min(v_min, value(newGameState, ghost_index + 1, current_depth, alpha, beta))
                if v_min < alpha: return v_min
                beta = min(beta, v_min)
            return v_min

        return maxValue(gameState, 1, -99999, 99999)[1]

class ExpectimaxAgent(MultiAgentSearchAgent):
    """
      Your expectimax agent (question 4)
    """

    def getAction(self, gameState):
        """
          Returns the expectimax action using self.depth and self.evaluationFunction

          All ghosts should be modeled as choosing uniformly at random from their
          legal moves.
        """
        "*** YOUR CODE HERE ***"

        def value(gameState, agent_index, current_depth):
            if agent_index == gameState.getNumAgents():
                agent_index = 0;
            # Choose Min or Max
            if agent_index == 0:
                return maxValue(gameState, current_depth + 1)[0]
            else:
                return expValue(gameState, agent_index, current_depth)

        def maxValue(gameState, current_depth):
            v_max = (-99999, 'Stop')
            pacman_actions = gameState.getLegalActions(0)
            # leaves
            if not pacman_actions or current_depth > self.depth:
                return (self.evaluationFunction(gameState), 'Stop')
            # Branches
            for pacman_action in pacman_actions:
                newGameState = gameState.generateSuccessor(0, pacman_action)
                v_max = max(v_max, (value(newGameState, 1, current_depth), pacman_action))
            return v_max

        def expValue(gameState, ghost_index, current_depth):
            v_exp = 0
            ghost_actions = gameState.getLegalActions(ghost_index)
            # leaves
            if not ghost_actions:
                return self.evaluationFunction(gameState)
            # Branches
            for ghost_action in ghost_actions:
                newGameState = gameState.generateSuccessor(ghost_index, ghost_action)
                p = 1.0 / len(ghost_actions)
                v_exp += p * value(newGameState, ghost_index + 1, current_depth)
            return v_exp

        return maxValue(gameState, 1)[1]

def betterEvaluationFunction(currentGameState):
    """
      Your extreme ghost-hunting, pellet-nabbing, food-gobbling, unstoppable
      evaluation function (question 5).

      DESCRIPTION: <write something here so we know what you did>
      summing all features: score, ghost points, ghost-buster points and
      distance points which is a search function to the closest food or capsule
    """
    "*** YOUR CODE HERE ***"

    pacman_pos = currentGameState.getPacmanPosition()
    food = currentGameState.getFood()
    capsules = currentGameState.getCapsules();
    ghosts = currentGameState.getGhostStates()
    scaredTimes = [ghost.scaredTimer for ghost in ghosts]

    # ghost points and ghost buster points
    g_points = -2
    b_points = 0
    ghost_lst = currentGameState.getGhostPositions()
    if ghost_lst:
        g_dis = min([[util.manhattanDistance(pacman_pos, ghost), ghost_index] for ghost_index, ghost in enumerate(ghost_lst)])
        if not g_dis[0]: g_dis[0] = 1
        g_points = g_points * (1.0 / g_dis[0])
        if g_dis[0] < 2: g_points = -20.0 / g_dis[0]
        if scaredTimes[g_dis[1]]:
            g_points = 0
            b_points = 10.0 * (1.0 / g_dis[0]) * (scaredTimes[g_dis[1]] / 40.0)

    # distance points
    d_points = 4
    problem = searchAgents.AnyFoodSearchProblem2(currentGameState)
    d_points = d_points * (1.0 / len(search.bfs(problem)))

    return currentGameState.getScore() + g_points + b_points + d_points

# Abbreviation
better = betterEvaluationFunction
