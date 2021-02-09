# -*- coding: utf-8 -*-
"""
Created on Sat Jan  9 00:01:02 2021

@author: Shadi and Noor
"""

import ASTAR

# ucs_search algorithm
def ucs_search(dim , startPoint , goalPoint , matrix,runningTimeAllowed,start):
    
    # Calling A* algorithim with a constant Heuristic value
    constHeuristicMatrix = dim * [(dim * [0])]
    return ASTAR.astar_search(dim, startPoint, goalPoint, matrix, constHeuristicMatrix,runningTimeAllowed,start)
