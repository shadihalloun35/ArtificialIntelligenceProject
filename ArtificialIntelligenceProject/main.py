# -*- coding: utf-8 -*-
"""
Created on Wed Jan  6 13:24:36 2021

@author: Shadi and Noor
"""

import ASTAR
#import UCS

from HeuristicFunctions import calcEuclideanHeuristic

import timeit

start = timeit.default_timer()

#Your statements here


f = open("secondInput.txt", "r")
savedInput  = f.read().splitlines()
algorithm = savedInput[0]
dim = int(savedInput[1])
startPoint_list = savedInput[2].split(',')
startPoint = (int(startPoint_list[0]),int(startPoint_list[0]))
goalPoint_list = savedInput[3].split(',')
goalPoint = (int(goalPoint_list[0]),int(goalPoint_list[1]))
matrix = [[int(num) for num in line.split(',')] for line in savedInput[4:]]


euclideanHeuristicMatrix = calcEuclideanHeuristic(dim,matrix)

path,expandedNodes = ASTAR.astar_search(dim , startPoint , goalPoint , matrix, euclideanHeuristicMatrix)

stop = timeit.default_timer()

print(path,expandedNodes)

print('Time: ', stop - start)  


#path,expandedNodes = UCS.ucs_search(dim , startPoint , goalPoint , matrix)

#print(path,expandedNodes)



'''
import sys
output = open(sys.argv[1] + ".txt", "w")
myfile = open(sys.argv[1])
en_data = myfile.read()
print(en_data)
'''           


