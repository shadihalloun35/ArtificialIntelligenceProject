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
       # print(d)
        if dls_search(dim,startPoint, goalPoint, matrix,d): 
            return True
    return False




def dls_search( dim , currentPoint , goalPoint , matrix,limit):
    
     visited = initDict(dim)
     opened = []
   #  closed = []
     depth = 0

     opened.append(currentPoint)
     visited[currentPoint] = True
     
    # print(currentPoint)
    # print(goalPoint)
     if currentPoint == goalPoint : 
         return True
  
     # If reached the maximum depth, stop recursing. 
  
     while len(opened) > 0: 
         print(depth)
         if depth <= limit:
            current = opened.pop(0) 
            
            # visited[current] = False
            if current == goalPoint:
                print("Goal Node Found")
                return True
            else: 
                # Get neighbours
                neighbourPoints = getNeighbours(dim,currentPoint,matrix)
                for  neighbour in neighbourPoints:
                    
                    # try to visit a node in future, if not already been to it
                    if(not(visited[neighbour])):
                        opened.append(neighbour)
                        visited[neighbour] = True
                depth += 1            
         else:
            print("Not found within depth limit")
            return False


     return False
    
 
# Initilizing dictionary for visited nodes
def initDict(dim):
    
    myDict = dict()
    
    for i in range(0,dim):
        for j in range(0,dim):
            myDict[(i,j)] = False
    
    return myDict
    
    
    

    
    