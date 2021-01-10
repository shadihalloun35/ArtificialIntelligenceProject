# -*- coding: utf-8 -*-
"""
Created on Sat Jan  9 13:38:11 2021

@author: shadi
"""



from Node import Node
from ASTAR import getNeighbours

# IDS algorithm
def ids_search(maximum_depth , dim , startPoint , goalPoint , matrix):
    
    #Loop for d from 1 to infinity
    for d in range(maximum_depth): 
        print(d)
        if dls_search(dim,startPoint, goalPoint, matrix,d): 
            return True
    return False




def dls_search( dim , currentPoint , goalPoint , matrix,depth):
    
     if currentPoint == goalPoint : return True
  
     # If reached the maximum depth, stop recursing. 
     if depth <= 0 : return False
  
     # Get neighbours
     neighborPoints = getNeighbours(dim,currentPoint,matrix)
        
     # Recur for all the vertices adjacent to this vertex 
     for neighbor in neighborPoints: 
         if dls_search(dim , neighbor, goalPoint, matrix , depth-1): 
             return True
     return False
    
    
    
    
    
    
    
    

    
    