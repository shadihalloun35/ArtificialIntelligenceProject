# -*- coding: utf-8 -*-
"""
Created on Wed Jan  6 13:24:36 2021
@author: Shadi and Noor
"""

import ASTAR
import BIASTAR
import IDS
import IDASTAR
import math
import sys
#import UCS

from HeuristicFunctions import calcEuclideanHeuristic
#from HeuristicFunctions import calcBackwardEuclideanHeuristic

import timeit

start = timeit.default_timer()

#Your statements here


f = open("firstInput.txt", "r")
savedInput  = f.read().splitlines()
algorithm = savedInput[0]
dim = int(savedInput[1])
startPoint_list = savedInput[2].split(',')
startPoint = (int(startPoint_list[0]),int(startPoint_list[0]))
goalPoint_list = savedInput[3].split(',')
goalPoint = (int(goalPoint_list[0]),int(goalPoint_list[1]))
matrix = [[int(num) for num in line.split(',')] for line in savedInput[4:]]
#goalPoint = (4,4)
#startPoint = (1,1)

euclideanHeuristicMatrix = calcEuclideanHeuristic(goalPoint,dim,matrix)
#print(euclideanHeuristicMatrix)
path,expandedNodes = ASTAR.astar_search(dim , startPoint , goalPoint , matrix, euclideanHeuristicMatrix)

#stop = timeit.default_timer()

#print(path)
#print('Time: ', stop - start)  


#path,expandedNodes = UCS.ucs_search(dim , startPoint , goalPoint , matrix)

#print(path,expandedNodes)

#path = IDS.ids_search(sys.maxsize , dim , startPoint , goalPoint , matrix)

#print(path)

ForwardEuclideanHeuristicMatrix = calcEuclideanHeuristic(goalPoint,dim,matrix)
BackwardEuclideanHeuristicMatrix = calcEuclideanHeuristic(startPoint,dim,matrix)
# =============================================================================
#print(ForwardEuclideanHeuristicMatrix)
#print("-----------------------------")
#print(BackwardEuclideanHeuristicMatrix)
# =============================================================================


#path,totalSumG = BIASTAR.biastar_search(dim , startPoint , goalPoint , matrix, ForwardEuclideanHeuristicMatrix,BackwardEuclideanHeuristicMatrix)
#print(path)
#print(totalSumG)

#path,expandedNodes = UCS.ucs_search(dim , startPoint , goalPoint , matrix)



path = IDASTAR.idastar_search(sys.maxsize, dim, startPoint, goalPoint, matrix, euclideanHeuristicMatrix)

print(path)





'''
import sys
output = open(sys.argv[1] + ".txt", "w")
myfile = open(sys.argv[1])
en_data = myfile.read()
print(en_data)
'''           