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
import signal
import matplotlib.pyplot as plt


class TimeoutError(Exception):
    pass

def timeout_handler(signum, frame):
    raise TimeoutError("Timeout occurred")

def main():
    print("-----------------------------------------------------------------")
    print("----------------- Bienvenue dans le Wumpus Game -----------------")
    print("-----------------------------------------------------------------")

    signal.signal(signal.SIGALRM, timeout_handler)

    wumpus = Wumpus()

    # Algorithme A* sur la grille de départ
    astar = AStar(wumpus, wumpus.heuristic)

    start_time = time()
    try:
        signal.alarm(10)  # Timeout de 10 secondes
        solution = astar.solve()
        execution_time_astar = time() - start_time
        signal.alarm(0)  # Annuler le timeout

        solution_cost = astar.solution_cost

        print("Grille de départ du jeu :\n")
        state = wumpus.getInitialState()
        wumpus.display_wumpus(state)

        print("\n------------- ALGO A* ---------------")
        print("\nChemin de la solution A* :", solution)
        print("Coût :", solution_cost)
        print("Temps d'exécution A* :", execution_time_astar, "secondes")
    except TimeoutError:
        print("Temps d'exécution A* dépassé (timeout)")

    # Algorithme IDA* sur la grille de départ
    IDAstar = IDAStar(wumpus, wumpus.heuristic)

    start_time = time()
    try:
        signal.alarm(10)  # Timeout de 10 secondes
        solution_path = IDAstar.solve()
        execution_time_idastar = time() - start_time
        signal.alarm(0)  # Annuler le timeout

        if solution_path == "echec":
            print("Aucune solution trouvée.")
        elif solution_path == "found":
            print("Solution trouvée.")
        else:
            print("\n------------ ALGO IDA* --------------")
            print("\nChemin de la solution:", solution_path)
            print("Coût :", IDAstar.solution_cost)
            print("Temps d'exécution IDA*:", execution_time_idastar, "secondes")
            print("\n\n")
    except TimeoutError:
        print("Temps d'exécution IDA* dépassé (timeout)")


    print("-----------------------------------------------------------------")
    print("--------------- Génération des grilles aléatoires ---------------")

    sizes = []
    num_traps = []
    execution_times_astar = []
    execution_times_idastar = []


    for _ in range(5):  # Exécutez le processus de génération aléatoire plusieurs fois pour différentes tailles de labyrinthe
        wumpus = Wumpus()
        wumpus.generate_random_instance()

        sizes.append(wumpus.n)
        num_traps.append(len(wumpus.trap_positions))

        astar = AStar(wumpus, wumpus.heuristic)

        start_time = time()
        try:
            signal.alarm(5)  # Timeout de 5 secondes
            solution = astar.solve()
            execution_time_astar = time() - start_time
            signal.alarm(0) 

            solution_cost = astar.solution_cost

            print("Chemin de la solution A* :", solution, ",", solution_cost, ")")
            execution_times_astar.append(execution_time_astar)
        except TimeoutError:
            print("Temps d'exécution A* dépassé pour cette grille (timeout)")
            execution_times_astar.append(None)

        idastar = IDAStar(wumpus, wumpus.heuristic)

        start_time = time()
        try:
            signal.alarm(20)  # Timeout de 20 secondes
            solution_path = idastar.solve()
            execution_time_idastar = time() - start_time
            signal.alarm(0)

            print("Chemin de la solution IDA* :", solution_path)
            execution_times_idastar.append(execution_time_idastar)
        except TimeoutError:
            print("Temps d'exécution IDA* dépassé pour cette grille (timeout)")
            execution_times_idastar.append(None)

    # Génération du premier graphique (A*)
    fig, ax1 = plt.subplots(figsize=(6, 6))
    scatter_astar = ax1.scatter(num_traps, execution_times_astar, c=sizes, cmap='viridis')
    ax1.set_xlabel('Nombre de pièges')
    ax1.set_ylabel("Temps d'exécution A* (secondes)")
    ax1.set_title("Temps d'exécution des instances avec l'algorithme A*")

    cbar = plt.colorbar(scatter_astar, ax=ax1, label='Taille du labyrinthe')    
    plt.show()

    # Génération du deuxième graphique (IDA*)
    fig, ax2 = plt.subplots(figsize=(6, 6))
    scatter_idastar = ax2.scatter(num_traps, execution_times_idastar, c=sizes, cmap='viridis')
    ax2.set_xlabel('Nombre de pièges')
    ax2.set_ylabel("Temps d'exécution IDA* (secondes)")
    ax2.set_title("Temps d'exécution des instances avec l'algorithme IDA*")

    cbar = plt.colorbar(scatter_idastar, ax=ax2, label='Taille du labyrinthe')

    plt.show()


if __name__ == '__main__':
    main()
