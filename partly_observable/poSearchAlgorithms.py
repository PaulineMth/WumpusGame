#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
This module contains the different classes to run the search algorithms in a partially observable environment. 

@author: Hugo Gilbert
"""


class AndOrSearch:
    """ Classe pour exécuter l'algorithme AndOrSearch. """

    def __init__(self, po_problem):
        """
        Parameters
        ----------
        po_problem : POProblem
            Le problème partiellement observable à résoudre
        """
        self.po_problem = po_problem

    def solve(self):
        initial_state = self.po_problem.getInitialState()
        return self.orSearch(initial_state, [])

    def orSearch(self, b_state, path):
        if self.po_problem.isFinal(b_state):
            return []

        if b_state in path:
            return 'echec'

        for action in self.po_problem.actions(b_state):
            possible_next_states = self.po_problem.update(b_state, action)
            result = self.andSearch(possible_next_states, [b_state]+ path)
            if result != 'echec':
                return [action]+ result
        return 'echec'

    def andSearch(self, b_states, path):
        for b_state in b_states:
            plan = self.orSearch(b_state, path)
            if plan == 'echec':
                return 'echec'
        return plan


class AOStar:
    """ Class to run the AndOrSearch algorithm.
    This class and this documentation has to be completed.
    """
    
    def __init__(self, po_problem, heuristic):
        """
        Parameters
        ----------
        po_problem : POProblem
            The partially observable problem to be solved. 
        heuristic : function
            The heuristic function to guide the search.
        """
        
        self.po_problem = po_problem
        self.heuristic = heuristic
        
        
    def solve(self):
        initial_state = self.po_problem.getInitialState()
        return self.aoSearch(initial_state, [])

    def aoSearch(self, b_state, path):
        if self.po_problem.isFinal(b_state):
            return []

        if b_state in path:
            return 'failure'

        path.append(b_state)
        actions = self.po_problem.actions(b_state)
        plan = None
        best_cost = float('inf')
        for action in actions:
            possible_states = self.po_problem.update(b_state, action)
            for possible_state in possible_states:
                h_value = self.heuristic(possible_state)
                cost = len(path) + h_value
                if cost < best_cost:
                    and_result = self.aoSearch(possible_state, path)
                    if and_result != 'failure':
                        plan = [action] + and_result
                        best_cost = cost
        path.remove(b_state)

        if plan is None:
            return 'failure'
        return plan

