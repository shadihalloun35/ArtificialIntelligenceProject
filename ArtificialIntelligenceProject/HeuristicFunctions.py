# -*- coding: utf-8 -*-
"""
Created on Fri Jan  8 15:52:16 2021
@author: Shadi and Noor
"""

import math

# Heuristic Function #1 using Euclidean Distance
def calcEuclideanHeuristic(goal,dim , matrix):
   euclideanHeuristicMatrix = []
    
   for i in range(1,dim+1):
       euclideanRow = []
       for j in range (1,dim+1):
           x = goal[0]+1 - i
           y = goal[1]+1 - j
           
           #Pythagoras Formula
           euclideanDistance = math.sqrt(x**2 + y**2)
           euclideanRow.append(euclideanDistance)
           
       euclideanHeuristicMatrix.append(euclideanRow)

   return euclideanHeuristicMatrix

'''   
def calcBackwardEuclideanHeuristic(start,goal,dim , matrix):
   euclideanHeuristicMatrix = []

   for i in range(dim,0,-1):
       euclideanRow = []
       for j in range (dim,0,-1):
           x = start[0]+1 - i
           y = start[1]+1 - j
           
           #Pythagoras Formula
           euclideanDistance = math.sqrt(x**2 + y**2)
           euclideanRow.append(euclideanDistance)
           
       euclideanHeuristicMatrix.append(euclideanRow)

   return euclideanHeuristicMatrix
'''