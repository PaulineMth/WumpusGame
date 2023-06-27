from observable.problem import Wumpus
from observable.searchAlgorithms import AStar
from observable.searchAlgorithms import IDAStar

from partly_observable.poProblem import POWumpus
from partly_observable.poSearchAlgorithms import AndOrSearch


def main():

    
    # Création d'un problème partiellement observable Wumpus
    po_problem = POWumpus()

    # Création de l'algorithme AndOrSearch
    and_or_search = AndOrSearch(po_problem)

    # Résolution du problème
    solution = and_or_search.solve()

    # Affichage de la solution
    if solution is not None:
        print("Solution trouvée :")
        for action in solution:
            print(action)
    else:
        print("Aucune solution trouvée.")

    """
    # Instanciez la classe AndOrSearch en lui passant votre problème partiellement observable en tant qu'argument
    andor_search = AndOrSearch(powumpus)

    # Exécutez l'algorithme AndOrSearch pour trouver la solution du problème
    solution = andor_search.solve()

    # Vérifiez si une solution a été trouvée
    if solution:
        print("Solution found:")
        for step in solution:
            print(step)
    else:
        print("No solution found.")
    """
        
if __name__ == "__main__":
    main()
