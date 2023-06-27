#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Simple Wumpus test in observable environment

@author: Hugo Gilbert
"""

from observable.problem import Wumpus
from observable.searchAlgorithms import AStar
from observable.searchAlgorithms import IDAStar
import numpy as np
from time import time

def main():
    wumpus = Wumpus()

    def display_wumpus(wumpus, state):
        maze = wumpus.maze.copy()

        maze[state.position] = 'A' #A for Alan
        maze = np.rot90(maze)

        for row in maze:
            print('  '.join(row))
    
    astar = AStar(wumpus, wumpus.heuristic)

    start_time = time()
    solution = astar.solve()
    end_time = time()

    execution_time_astar = end_time - start_time
    solution_cost = astar.solution_cost

    # Affichez la solution obtenue
    print("------------- ALGO A* ---------------")
    print("Solution A*:", solution_cost)
    print("Solution A*:", solution)
    print("Temps d'exécution A*:", execution_time_astar, "secondes")

    state = wumpus.getInitialState() 
    display_wumpus(wumpus, state)

    IDAstar = IDAStar(wumpus, wumpus.heuristic)

    start_time = time()
    solution_path, solution_cost = IDAstar.solve()
    end_time = time()

    execution_time_idastar = end_time - start_time

    # Étape 4: Afficher la solution
    if solution_path == "failure":
        print("Aucune solution trouvée.")
    elif solution_path == "found":
        print("Solution trouvée.")
    else:
        print("------------- ALGO IDA* ---------------")
        print("Coût de la solution:", solution_cost)
        print("Chemin de la solution:", solution_path)
        print("Temps d'exécution IDA*:", execution_time_idastar, "secondes")


    wumpus.generate_random_instance()
    state = wumpus.getInitialState()  # Obtenir le nouvel état initial
    display_wumpus(wumpus, state)

    astar = AStar(wumpus, wumpus.heuristic)

    start_time = time()
    solution = astar.solve()
    end_time = time()

    execution_time_astar = end_time - start_time
    solution_cost = astar.solution_cost

    # Affichez la solution obtenue
    print("------------- ALGO A* ---------------")
    print("Solution A*:", solution_cost)
    print("Solution A*:", solution)
    print("Temps d'exécution A*:", execution_time_astar, "secondes")


if __name__ == '__main__':
    main()
