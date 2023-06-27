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


def main():
    poWumpus = POWumpus()

    andOrSearch = AndOrSearch(poWumpus)

    start_time = time()
    solutionAndOrSearch = andOrSearch.solve()
    end_time = time()

    execution_time_andor = end_time - start_time

    # Affichage de la solution
    if solutionAndOrSearch is not None:
        print("Solution trouvée :")
        for action in solutionAndOrSearch:
            print(action)
    else:
        print("Aucune solution trouvée.")

    # aoStar = AOStar(problem, problem.heuristic)
    # print(aoStar.solve())

    print("Temps d'exécution And-Or Search:", execution_time_andor, "secondes")


if __name__ == '__main__':
    main()
