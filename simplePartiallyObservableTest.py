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


def show_grid(actions, maze):
    state = list(maze.getInitialState())[0]  # Obtenir l'état initial à partir de l'objet maze

    # Appliquer toutes les actions à l'état
    for action in actions:
        state = maze.problem.transition(state, action)

    # Afficher la grille finale
    print("Grille finale :")
    print_grid(maze, state)

def print_grid(maze, state):
    # Obtenir la grille actuelle à partir de l'objet maze
    grid = maze.problem.maze.copy()

    # Mettre à jour la grille avec l'état actuel
    grid[state.position] = 'A'  # En supposant que 'A' représente l'agent

    # Faire pivoter la grille pour aligner les actions avec la grille imprimée
    grid = np.rot90(grid)

    # Afficher la grille sous forme de représentation visuelle
    for row in grid:
        print(' '.join(row))




def main():

    poWumpus = POWumpus()
    andOrSearch = AndOrSearch(poWumpus)
    start_time = time()
    solutionAndOrSearch = andOrSearch.solve()
    end_time = time()

    execution_time_andor = end_time - start_time

    if solutionAndOrSearch is not None:
        print("-------------AOrSearch--------------- :")
        print("Solution trouvée :", solutionAndOrSearch)
        show_grid(solutionAndOrSearch, poWumpus)  # Afficher la grille fina  # Afficher la grille
    else:
        print("Aucune solution trouvée.")
    print("Temps d'exécution:", execution_time_andor, "secondes")


    poWumpus = POWumpus()
    andOrStar = AOStar(poWumpus ,poWumpus.heuristic)
    start_time = time()
    solutionAndOrStar = andOrStar.solve()
    end_time = time()

    execution_time_andor = end_time - start_time

    # Affichage de la solution
    if solutionAndOrStar is not None:
        print("-------------AO*--------------- :")
        print("Solution trouvée :", solutionAndOrStar)
        show_grid(solutionAndOrSearch, poWumpus)  # Afficher la grille fina
    else:
        print("Aucune solution trouvée.")
    print("Temps d'exécution:", execution_time_andor, "secondes")


    


if __name__ == '__main__':
    main()
