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

# Heuristic Function #2 using Manhattan Distance
def calcManhattanHeuristic(goal,dim , matrix):
   manhattanHeuristicMatrix = []
    
   for i in range(1,dim+1):
       manhattanRow = []
       for j in range (1,dim+1):
           deltax = abs(goal[0]+1 - i)
           deltay = abs(goal[1]+1 - j)
           
           #Manhattan Formula
           manhattanDistance = (deltax + deltay)/(dim+1)
           manhattanRow.append(manhattanDistance)
           
       manhattanHeuristicMatrix.append(manhattanRow)

   return manhattanHeuristicMatrix


# Heuristic Function #3 using Cosine Similarity
def calcCosineHeuristic(goal,dim , matrix):
   cosineHeuristicMatrix = []
    
   for i in range(0,dim):
       cosineRow = []
       for j in range (0,dim):
           
           # Dot Product of Two vectors
           numerator = i * goal[0] + j * goal[1]
           
           # The sum of the length of the vectors
           denominator  = math.sqrt(i**2 + j**2) * math.sqrt(goal[0]**2 + goal[1]**2)
           
           #Cosine Similarity Formula
           if denominator != 0:
               cosineSimilarity = numerator/denominator
           else:
               cosineSimilarity = math.sqrt(goal[0]**2 + goal[1]**2)
               
           cosineDistance = (math.cos(cosineSimilarity))/math.pi
           cosineRow.append(cosineDistance)
           
       cosineHeuristicMatrix.append(cosineRow)

   return cosineHeuristicMatrix
