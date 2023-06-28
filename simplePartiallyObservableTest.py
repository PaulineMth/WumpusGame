#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Simple Wumpus test in partially observable environment

@author: Hugo Gilbert
"""

from partly_observable.poProblem import POWumpus
from partly_observable.poSearchAlgorithms import AndOrSearch
from partly_observable.poSearchAlgorithms import AOStar
import numpy as np
from time import time


def display_InitalStates(maze, states):
    for state in states:
        grid = maze.problem.maze.copy()
        grid[state.position] = 'A'  # A for Alan
        grid = np.rot90(grid)
        for row in grid:
            print(' '.join(row))
        print()

def display_FinalPoWumpus(actions, maze):
    state = list(maze.getInitialState())[0]  # Obtenir l'état initial à partir de l'objet maze

    for action in actions:
        state = maze.problem.transition(state, action)

    print("Grille finale")
    display_PoWumpus(maze, state)

def display_PoWumpus(maze, state):
    grid = maze.problem.maze.copy()
    grid[state.position] = 'A'  # A for Alan
    grid = np.rot90(grid)
    for row in grid:
        print(' '.join(row))

def main():
    poWumpus = POWumpus()
    print("--------------------------------------------------------------")
    print("--------------- Grilles de départ potentielles ---------------")
    display_InitalStates(poWumpus, poWumpus.getInitialState())
    andOrSearch = AndOrSearch(poWumpus)
    start_time = time()
    solutionAndOrSearch = andOrSearch.solve()
    end_time = time()

    execution_time_andor = end_time - start_time

    if solutionAndOrSearch is not None:
        print("---------------- AOrSearch ------------------\n")
        print("Solution trouvée :", solutionAndOrSearch)
    else:
        print("Aucune solution trouvée.")
    print("Temps d'exécution:", execution_time_andor, "secondes\n")
    display_FinalPoWumpus(solutionAndOrSearch, poWumpus) 

    poWumpus = POWumpus()
    andOrStar = AOStar(poWumpus ,poWumpus.heuristic)
    start_time = time()
    solutionAndOrStar = andOrStar.solve()
    end_time = time()

    execution_time_andor = end_time - start_time

    if solutionAndOrStar is not None:
        print("\n------------------- AO* ---------------------\n")
        print("Solution trouvée :", solutionAndOrStar)
    else:
        print("Aucune solution trouvée.")
    print("Temps d'exécution:", execution_time_andor, "secondes\n")
    display_FinalPoWumpus(solutionAndOrSearch, poWumpus) 

if __name__ == '__main__':
    main()
