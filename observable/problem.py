#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
This module contains an abstract class to represent a search problem. 
This class in inherited by a simple Wumpus class to test your code.  
@author: Hugo Gilbert
"""
from abc import ABC, abstractmethod
import numpy as np
import math

class Problem(ABC):
    """
    An astract class to represent a search problem.
    
    Methods
    -------
    actions(state)
        Yields the set of valid actions
    transition(state, action)
        The transtion function of the search problem
    isFinal(state)
        Asserts if a state is final or not
    actionCost(state, action)
        The cost function of the search problem
    getInitialState()
        Yields the initial state of the search problem
    """
    
    @abstractmethod    
    def actions(self, state):
        """ Yields the set of valid actions.
        Parameters
        ----------
        state : State
            a given state

        Returns
        -------
        set
            a set of chars representing the valid actions in the given state     
        """

        pass
    
    
    @abstractmethod
    def transition(self, state, action):
        """ The transtion function of the search problem.
        Parameters
        ----------
        state : State 
            a given state  
        action : char
            a valid action in the state

        Returns
        -------
        the state obtained when performing action in state
        """

        pass
    

    @abstractmethod
    def isFinal(self, state):
        """ Asserts if a state is final or not.

        Parameters
        ----------
        state : State
            a given state

        Returns
        -------
        True if the state is final else False

        """
        
        pass
    
    @abstractmethod
    def actionCost(self, state, action):
        """ The cost function of the search problem.
        Parameters
        ----------
        state : State
            a given state  
        action : char
            a valid action in the state

        Returns
        -------
        the cost (a positive integer) incurred when performing action in state
        """
        
        pass
    
    @abstractmethod
    def getInitialState(self):
        """ Yields the initial state of the search problem.
        Returns
        -------
        the initial state of the search problem
        """
        
        pass
    

class Wumpus(Problem):    
    """
    A class to define a simple Wumpus problem.
    
    
    Attributes
    ----------
    ELEMENTS : dict
        Keys are strings and Values are chars. Used to described the different elements in the maze
    ACTIONS : dict
        Keys are strings and Values are chars. Used to described the different possible actions
    n : int
       The maze in an n * n grid
    wumpus_position: (int,int)
       The position of the wumpus in the maze
    treasure_position: (int,int)
       The position of the treasure in the maze
    maze : numpy.array
       An array of char representing the maze
       
    Methods
    -------
    actions(state)
        Yields the set of valid actions
    transition(state, action)
        The transtion function of the search problem
    isFinal(state)
        Asserts if a state is final or not
    actionCost(state, action)
        The cost function of the search problem
    getInitialState()
        Yields the initial state of the search problem
    heuristic(state)
        An heuristic function to guide the search
    """
    
    ELEMENTS = {"EMPTY" : 'E', "TREASURE" : 'T', "SNARE" : 'S', "WUMPUS": 'W'}
    ACTIONS = {"LEFT" : 'L', "RIGHT" : 'R', "UP" : 'U', "DOWN": 'D', "MAGIC": 'M'}
    
    
    def __init__(self):
        self.n = 4
        self.wumpus_position = (3,2)
        self.treasure_position = (2,3)
        self.maze = np.empty((self.n,self.n), np.dtype(str))
        self.maze[:] = Wumpus.ELEMENTS["EMPTY"]
        self.maze[1,2] = Wumpus.ELEMENTS["SNARE"]
        self.maze[2,2] = Wumpus.ELEMENTS["SNARE"]
        self.maze[1,3] = Wumpus.ELEMENTS["SNARE"]
        self.maze[2,0] = Wumpus.ELEMENTS["SNARE"]
        self.maze[self.treasure_position] = Wumpus.ELEMENTS["TREASURE"]
        self.maze[self.wumpus_position] = Wumpus.ELEMENTS["WUMPUS"]
        
    class WumpusState:
        """
        Inner state class to define a state in the Wumpus problem.
        
        Attributes
        ----------
        position : (int, int)
            a position in the maze
        wumpus_beaten : bool
            True if the Wumpus is defeated else False
        """
        
        def __init__(self,pos = (0,0),wb = False):
            """
            Parameters
            ----------
            pos : (int, int), optional
                a position in the maze. The default is (0,0).
            wb : bool, optional
                True if the Wumpus is defeated else False. The default is False.
            """
            self.position = pos 
            self.wumpus_beaten = wb
            
        def __str__(self):
            return str(self.position) + " " + str(self.wumpus_beaten)
        
        def __hash__(self):
            return hash(self.position) + hash(self.wumpus_beaten)
        
        def __eq__(self, other):
            return (self.position == other.position) and self.wumpus_beaten == other.wumpus_beaten
        
        def __lt__(self,other):
            return (self.position,self.wumpus_beaten) < (other.position, other.wumpus_beaten)

         
    def actions(self, state):
        """ Yields the set of valid actions. The agent may move if it is not trapped by a snare or the Wumpus and may use magic if the Wumpus is close. 
        Parameters
        ----------
        state : State
            a given state

        Returns
        -------
        set
            a set of chars representing the valid actions in the given state     
        """
        
        posx, posy = state.position
        res = set()
        if (self.maze[posx,posy] == Wumpus.ELEMENTS["SNARE"])\
            or ((self.maze[posx,posy] == Wumpus.ELEMENTS["WUMPUS"]) and (state.wumpus_beaten == False)):
            return res
        if(posx!=0):
            res.add(Wumpus.ACTIONS["LEFT"])
        if(posx!=self.n-1):
            res.add(Wumpus.ACTIONS["RIGHT"])
        if(posy!=0):
            res.add(Wumpus.ACTIONS["DOWN"])
        if(posy!=self.n-1):
            res.add(Wumpus.ACTIONS["UP"])
        if(((abs(posx-self.wumpus_position[0])+abs(posy-self.wumpus_position[1])) == 1) and (state.wumpus_beaten == False)):
            res.add(Wumpus.ACTIONS["MAGIC"])                           
        return res
    
    def transition(self, state, action):
        """ The transtion function of the Wumpus search problem.
        Parameters
        ----------
        state : State 
            a given state  
        action : char
            a valid action in the state

        Returns
        -------
        the state obtained when performing action in state
        """
        
        posx, posy = state.position
        wb = state.wumpus_beaten
        if action == Wumpus.ACTIONS["LEFT"]:
            return self.WumpusState((posx-1,posy),wb)
        elif action == Wumpus.ACTIONS["RIGHT"]:
            return self.WumpusState((posx+1,posy),wb)
        elif action == Wumpus.ACTIONS["DOWN"]:
            return self.WumpusState((posx,posy-1),wb)
        elif action == Wumpus.ACTIONS["UP"]:
            return self.WumpusState((posx,posy+1),wb)
        elif action == Wumpus.ACTIONS["MAGIC"]:
            return self.WumpusState((posx,posy),True)
        raise Exception("Invalid action")
        
    def isFinal(self, state):
        """ Asserts if a state is final or not. A state is final if the position matches the one of the Treasure and if the Wumpus is defeated.

        Parameters
        ----------
        state : State
            a given state

        Returns
        -------
        True if the state is final else False

        """
        
        return state.position == self.treasure_position and state.wumpus_beaten
        
    def actionCost(self, state, action):
        """ The cost function of the Wumpus search problem.
        Parameters
        ----------
        state : State 
            a given state  
        action : char
            a valid action in the state

        Returns
        -------
        the cost (a positive integer) incurred when performing action in state, 1 for a move action and 5 to use magic
        """
        
        return 5 if action == Wumpus.ACTIONS["MAGIC"] else 1
        
    def getInitialState(self):
        """ Yields the initial state of the Wumpus search problem.
        Returns
        -------
        the initial state of the search problem, by default in (0,0) and the Wumpus alive
        """
        
        return self.WumpusState()
    
    def generate_random_instance(self):
        """
        Generates a random instance of the Wumpus search problem.

        This method randomly generates the positions of the Wumpus, the treasure, and the traps in the maze.

        Returns
        -------
        None
         """
        self.n = np.random.randint(4, 9)

        # Générer aléatoirement les positions du Wumpus et du trésor
        self.wumpus_position = (np.random.randint(self.n), np.random.randint(self.n))
        self.treasure_position = (np.random.randint(self.n), np.random.randint(self.n))

        # Générer les positions des pièges
        min_num_traps = int(0.1 * self.n * self.n)  # Nombre minimum de pièges
        max_num_traps = int(0.2 * self.n * self.n)  # Nombre maximum de pièges
        num_traps = np.random.randint(min_num_traps, max_num_traps + 1)
        trap_positions = []
        self.trap_positions = []
        while len(self.trap_positions) < num_traps:
            trap_x = np.random.randint(self.n)
            trap_y = np.random.randint(self.n)
            if (trap_x, trap_y) != self.wumpus_position and (trap_x, trap_y) != self.treasure_position and (trap_x, trap_y) not in self.trap_positions:
                self.trap_positions.append((trap_x, trap_y))

        # Placer le Wumpus, le trésor et les pièges dans le labyrinthe
        self.maze = np.full((self.n, self.n), Wumpus.ELEMENTS["EMPTY"])
        for trap_pos in self.trap_positions:
            self.maze[trap_pos] = Wumpus.ELEMENTS["SNARE"]
        self.maze[self.treasure_position] = Wumpus.ELEMENTS["TREASURE"]
        self.maze[self.wumpus_position] = Wumpus.ELEMENTS["WUMPUS"]

        print("\nn :", self.n)
        print("Nombre de pièges :", num_traps)
        initial_state = self.getInitialState()
        self.display_wumpus(initial_state)

            
    def heuristic(self, state):
        """Une fonction heuristique qui calcule la distance en ligne droite entre toutes les positions possibles et la position du trésor.

        Parameters
        ----------
        state : State 
            a given state

        Returns
        -------
        float
            La valeur de l'heuristique (distance en ligne droite)
        """
        treasure_x, treasure_y = self.treasure_position
        pos_x, pos_y = state.position

        straight_line_distance = math.sqrt((treasure_x - pos_x)**2 + (treasure_y - pos_y)**2)

        return straight_line_distance
    
    def display_wumpus(self, state):
        """ Display the maze
        Parameters
        ----------
        state : State 
            a given state
        """
        maze = self.maze.copy()

        maze[state.position] = 'A'  # A for Alan
        maze = np.rot90(maze)

        for row in maze:
            print('  '.join(row))

        
    
    