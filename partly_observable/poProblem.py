#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
This module contains the classes to represent a partially observable Wumpus problem.
@author: Hugo Gilbert
"""
from abc import ABC, abstractmethod
from observable.problem import Wumpus

class POProblem(ABC):
    """
    Abstract class to represent a general artially observable problem
    
    Attributes
    ----------
    problem : Problem
        The physical problem the partially observable problem is built on.
    
    Methods
    -------
    actions(beliefState)
        Returns the valid actions given a belief-state
    prediction(beliefState, action)
        Returns the set of possible future states given a belief-state and an action
    percepts(state)
        Abstract method which should return the percepts that the agent observes in a given state 
    possiblePercepts(beliefState)
        Returns the set of possible percepts that can be observed given a belief-state
    update(states, action)
        Returns the set of possible future states given a belief-state and an action partitioned with respect to the percepts that would be observed
    isFinal(beliefState)
        Returns True if the beliefState is final else False
    actionCost(beliefState, action)
        Returns the cost of performing an action in a given belief-state, i.e., the maximum possible cost.
    getInitialState()
        Abstract method which should return the initial belief-state.
    """
    
    def __init__(self, problem):
        """
        Parameters
        ----------
        problem : Problem
            The physical problem the partially observable problem is built on.

        """
        self.problem = problem
            
    def actions(self, beliefState):
        """ Returns the valid actions given a belief-state
        
        Parameters
        ----------
        beliefState : set
            a beliefState, i.e., a set of states.

        Returns
        -------
        set
           the set of valid actions in the given belief-state.

        """
        res = set()
        for state in beliefState:
            res |= self.problem.actions(state)
        return res 
    
    def prediction(self, beliefState, action):
        """ Returns the set of possible future states given a belief-state and an action.
        
        Parameters
        ----------
        beliefState : set
            A beliefState, i.e., a set of states.
        action : char
            A valid action given the belief-state.

        Returns
        -------
        set
            The set of possible future states when performing action in beliefState.
        """
        res = set()
        for state in beliefState:
            if action in self.problem.actions(state):
                res |= {self.problem.transition(state,action)}
            else: 
                res |= {state}
        return res
    
    @abstractmethod
    def percepts(self, state):
        """ Abstract method which should return the percepts that the agent receive in a given state.

        Parameters
        ----------
        state : State
            A state of the physical search problem.

        Returns
        -------
        set
            A set of percepts observed in the given state. 

        """
        pass
    
    def possiblePercepts(self, beliefState):
        """Returns the set of possible percepts that can be observed given a belief-state.
        
        Parameters
        ----------
        beliefState : set
            A beliefState, i.e., a set of states.

        Returns
        -------
        set
            The set of possible percepts that can be observed given beliefState.
        """
        return {frozenset(self.percepts(state)) for state in beliefState}
    
    def update(self, states, action):
        """ Returns the set of possible future states given a belief-state and an action partitioned with respect to the percepts that would be observed.
        
        Parameters
        ----------
        beliefState : set
            A beliefState, i.e., a set of states.
        action : char
            A valid action given the belief-state.

        Returns
        -------
        set
            The set of possible future states when performing action in beliefState partitioned with respect to the percepts that would be observed.

        """
        possibleNextStates = self.prediction(states, action)
        possibleNextPercepts = self.possiblePercepts(possibleNextStates)
        res = []
        for percept in possibleNextPercepts:
            res.append({state for state in possibleNextStates if self.percepts(state) == percept})
        return res
    
    def isFinal(self, beliefState):
        """ Returns True if the beliefState is final else False.

        Parameters
        ----------
        beliefState : set
            A beliefState, i.e., a set of states.

        Returns
        -------
        bool
            True if the beliefState is final else False.

        """
        for state in beliefState:
            if not self.problem.isFinal(state):
                return False
        return True
    

    def actionCost(self, beliefState, action):
        """ Returns the cost of performing an action in a given belief-state, i.e., the maximum possible cost.
        
        Parameters
        ----------
        beliefState : set
            A beliefState, i.e., a set of states.
        action : char
            A valid action given the belief-state.

        Returns
        -------
        int 
            the cost of performing action in beliefState.

        """
        res = 0
        for state in beliefState:
            res = max(res, self.problem.actionCost(state,action))
        return res
            
    
    @abstractmethod
    def getInitialState(self):
        """ Abstract method which should return the initial belief-state.

        Returns
        -------
        set
            the initial belief-state. 

        """
        pass
    
class POWumpus(POProblem):
    """ A class to represent a partially observable Wumpus problem.
    
    Attributes
    ----------
    PERCEPTS : dict
        Keys are strings and Values are chars. Used to described the different possible percepts the agent can receive  
    problem : Wumpus
        A Wumpus physical search problem
    
    Methods
    -------
    percepts(state)
        Returns the percepts that the agent observes in a given state
    getInitialState()
        Returns the initial belief-state
    heuristic(beliefState)
        An heuristic function to guide the search.
    
    """
    PERCEPTS = {"WallOnLeft" : 'L', "WallOnRight" : 'R', "WallOnDown" : 'D',  "WallOnUp" : 'U', "SnareClose" : 'S', "WumpusClose": 'W'}
    
    def __init__(self):
        super().__init__(Wumpus())  # Appel au constructeur de la classe parente
        self.maze = self.problem.maze  # Utilisation de l'attribut maze de la classe parente
        
    def percepts(self, state):
        """ Returns the percepts that the agent observes in a given state.

        Parameters
        ----------
        state : State
            A state of the physical Wumpus search problem.

        Returns
        -------
        set
            A set of percepts observed in the given state.
        """
        res = set()
        posx,posy = state.position
        if(posx==0):
            res.add(POWumpus.PERCEPTS["WallOnLeft"])
        elif(self.problem.maze[posx-1,posy] == Wumpus.ELEMENTS["SNARE"]):
                res.add(POWumpus.PERCEPTS["SnareClose"])
        if(posx==self.problem.n-1):
            res.add(POWumpus.PERCEPTS["WallOnRight"])
        elif(self.problem.maze[posx+1,posy] == Wumpus.ELEMENTS["SNARE"]):
                res.add(POWumpus.PERCEPTS["SnareClose"])
        if(posy==0):
            res.add(POWumpus.PERCEPTS["WallOnDown"])
        elif(self.problem.maze[posx,posy-1] == Wumpus.ELEMENTS["SNARE"]):
                res.add(POWumpus.PERCEPTS["SnareClose"])
        if(posy==self.problem.n-1):
            res.add(POWumpus.PERCEPTS["WallOnUp"])
        elif(self.problem.maze[posx,posy+1] == Wumpus.ELEMENTS["SNARE"]):
                res.add(POWumpus.PERCEPTS["SnareClose"])
        if(((abs(posx-self.problem.wumpus_position[0])+abs(posy-self.problem.wumpus_position[1])) == 1) and (state.wumpus_beaten == False)):
                res.add(POWumpus.PERCEPTS["WumpusClose"])
        return res
    
    def getInitialState(self):
        """ Returns the initial belief-state.

        Returns
        -------
        set
            The initial belief-state.

        """
        return {Wumpus.WumpusState(pos = (1,0)),Wumpus.WumpusState(pos = (3,0))}
    
    def heuristic(self, beliefState):
        """ An heuristic function to guide the search.
        
        Parameters
        ----------
        beliefState : set
            A beliefState, i.e., a set of states.
            
        Returns
        -------
        int
            The heuristic score of the current belief state, i.e., an estimation of the path cost that remains to be done in the worst case.
        
        """
        h = 0;
        for s in beliefState:
            hs = self.problem.heuristic(s)
            if hs > h:
                h = hs
        return h

