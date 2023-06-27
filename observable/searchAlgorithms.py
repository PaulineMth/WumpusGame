from observable.problem import Problem
from time import sleep
import heapq
"""
This module contains the different classes to run the search algorithms in an observable environment. 

@author: Hugo Gilbert
"""

class Node:
    """
    A class to define a node in the search tree. 
    
    Attributes
    ----------
    state : State
        A state of the search problem
    problem : Problem
        The search problem to be solved
    fatherNode : Node
        The father node in the search tree
    actionFromFather : char
        The action that led to this new node from the father node
    pathCost : int
        The cost of the path from the root to this node in the search tree
        
    Methods
    -------
    expand()
        Method to expand the node
    getSolution()
        Method to obtain the list of actions from the root to a terminal node 
        
    """
    def __init__(self, problem, state, fatherNode, actionFromFather):
        """
        Parameters
        ----------
        problem : Problem
            The search problem to be solved
        state : State
            A state of the search problem
        fatherNode : Node
            The father node in the search tree
        actionFromFather : char
            The action that led to this new node from the father node
        """
        
        self.state = state
        self.problem = problem
        self.fatherNode = fatherNode
        self.actionFromFather = actionFromFather
        self.pathCost = 0
        if(fatherNode):
            self.pathCost = fatherNode.pathCost + problem.actionCost(fatherNode.state, actionFromFather) 
    
    def __str__(self):
        return str(self.state)
    
    def __lt__(self,other):
        return self.state < other.state
    
    def expand(self):
        """ Method to expand the node. 
        Returns
        -------
        res : list
            The list of succesor nodes obtained from expanding the current node.
        """
        res = []
        for action in self.problem.actions(self.state):
            nextState = self.problem.transition(self.state,action)
            res.append(Node(self.problem, nextState, self, action))
        return res
            
    def getSolution(self):
        """ Method to obtain the list of actions from the root to a terminal node. 
        

        Returns
        -------
        res : []
            List of actions from the root to the terminal node.
        """
        res = []
        node = self
        while(node.fatherNode):
            res.append(node.actionFromFather)
            node = node.fatherNode
        res.reverse()
        return res

import heapq

class Frontier:
    """ Class to represent the frontier in the AStar algorithm.
    
    Attributes
    ----------
    frontier : list
        A list representing the frontier. It contains pairs where the first element is an AStar score and the second element is a node.    
    
    Methods
    -------
    push(score, node)
        Pushes a new node into the frontier.
    pop()
        Pops the search node with the smallest AStar score.
    isEmpty()
        Checks if the frontier is empty.
    isInFrontier(state)
        Checks if a state is already present in the frontier.
    getFrontierNodeCost(state)
        Returns the cost of a node in the frontier.
    replaceFrontierNode(state, cost)
        Replaces a node in the frontier with a new node of lower cost.
    """

    def __init__(self):
        self.frontier = []
    
    def __str__(self):
        return " ".join((str(x) + str(y)) for (x, y) in self.frontier)

    def push(self, score, node):
        """ Pushes a new node into the frontier. 
        If the node is already present, the method checks the current AStar score of the node to decide which one to keep.

        Parameters
        ----------
        score : int
            The AStar score of the node to be stored.
        node : Node
            The search node to be stored.

        """
        found = False
        for i in range(len(self.frontier)):
            (s, n) = self.frontier[i]
            if node.state == n.state:
                found = True
                if s > score:
                    self.frontier.pop(i)
                    heapq.heappush(self.frontier, (score, node))
                break
        if not found:
            heapq.heappush(self.frontier, (score, node))
        
    def pop(self):
        """ Pops the search node with the smallest AStar score. 

        Returns
        -------
        (int, Node)
            A pair with the search node with the smallest AStar score (in the second position) and the smallest AStar score (in the first position).
        """
        return heapq.heappop(self.frontier)
        
    def isEmpty(self):
        """ Checks if the frontier is empty.

        Returns
        -------
        bool
            True if the frontier is empty; False otherwise.
        """
        return len(self.frontier) == 0

    def isInFrontier(self, state):
        """ Checks if a state is already present in the frontier.

        Parameters
        ----------
        state : object
            The state to check.

        Returns
        -------
        bool
            True if the state is already present in the frontier; False otherwise.
        """
        for _, node in self.frontier:
            if node.state == state:
                return True
        return False

    def getFrontierNodeCost(self, state):
        """ Returns the cost of a node in the frontier.

        Parameters
        ----------
        state : object
            The state of the node to retrieve the cost for.

        Returns
        -------
        float
            The cost of the node in the frontier if found; float('inf') otherwise.
        """
        for cost, node in self.frontier:
            if node.state == state:
                return cost
        return float('inf')

    def replaceFrontierNode(self, state, cost):
        """ Replaces a node in the frontier with a new node of lower cost.

        Parameters
        ----------
        state : object
            The state of the node to replace.
        cost : float
            The new cost for the replacement node.
        """
        for i, (old_cost, node) in enumerate(self.frontier):
            if node.state == state and old_cost > cost:
                self.frontier[i] = (cost, node)
                heapq.heapify(self.frontier)
                break

    def show(self):
        """ Displays the contents of the frontier. """
        for score, node in self.frontier:
            #print("Score:", score)
            print("Node:", node)



class AStar:
    """ Class for the AStar Algorithm. """

    def __init__(self, problem, heuristic):
        """
        Parameters
        ----------
        problem : Problem
            The search problem to be solved.
        heuristic : function
            The heuristic function to guide the search.
        """
        self.problem = problem
        self.heuristic = heuristic
        self.solution_cost = None

    def solve(self):
        initial_state = self.problem.getInitialState()
        initial_node = Node(self.problem, initial_state, None, None)
        frontier = Frontier()
        exploredSet = set()

        frontier.push(initial_node.pathCost, initial_node)

        while True:
            if frontier.isEmpty():
                return "failure"

            node_cost, node = frontier.pop()

            if self.problem.isFinal(node.state):
                self.solution_cost = node.pathCost
                return node.getSolution()

            exploredSet.add(node.state)

            for action in self.problem.actions(node.state):
                child_state = self.problem.transition(node.state, action)
                child_cost = node.pathCost + self.problem.actionCost(node.state, action)

                if child_state not in exploredSet and not frontier.isInFrontier(child_state):
                    child_node = Node(self.problem, child_state, node, action)
                    child_score = child_cost + self.heuristic(child_state)
                    frontier.push(child_score, child_node)
                elif frontier.isInFrontier(child_state) and frontier.getFrontierNodeCost(child_state) > child_cost:
                    frontier.replaceFrontierNode(child_state, child_cost)


class IDAStar:
    """ Classe pour l'algorithme IDAStar. """

    def __init__(self, problem, heuristic):
        """
        Paramètres
        ----------
        problem : Problem
            Le problème de recherche à résoudre.
        heuristic : fonction
            La fonction heuristique pour guider la recherche.
        """
        self.problem = problem
        self.heuristic = heuristic
        self.solution_path = []
        self.solution_cost = float('inf')

    def solve(self):
        initial_state = self.problem.getInitialState()
        initial_node = Node(self.problem, initial_state, None, None)
        threshold = self.heuristic(initial_state)

        while True:
            result = self.search(initial_node, 0, threshold)
            if result == "found":
                return self.solution_path, self.solution_cost
            if result == float("inf"):
                return "failure", float('inf')
            threshold = result

    def search(self, node, g, threshold):
        f = g + self.heuristic(node.state)

        if f > threshold:
            return f

        if self.problem.isFinal(node.state):
            self.solution_path = node.getSolution()
            self.solution_cost = node.pathCost
            return "found"

        minimum = float("inf")
        for successor in node.expand():
            result = self.search(
                successor, g + self.problem.actionCost(node.state, successor.actionFromFather), threshold
            )
            if result == "found":
                return result
            if result < minimum:
                minimum = result

        return minimum
