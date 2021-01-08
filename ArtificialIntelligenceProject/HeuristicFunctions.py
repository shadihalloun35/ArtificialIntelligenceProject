# -*- coding: utf-8 -*-
"""
Created on Fri Jan  8 15:52:16 2021

@author: shadi
"""

import math

# Heuristic Function #1 using Euclidean Distance
def calcEuclideanHeuristic(dim , matrix):
   euclideanHeuristicMatrix = []
    
   for i in range(1,dim+1):
       euclideanRow = []
       for j in range (1,dim+1):
           x = dim - i
           y = dim - j
           
           #Pythagoras Formula
           euclideanDistance = math.sqrt(x**2 + y**2)
           euclideanRow.append(euclideanDistance)
           
       euclideanHeuristicMatrix.append(euclideanRow)

   return euclideanHeuristicMatrix
    