# -*- coding: utf-8 -*-
"""
Created on Sat Jan  9 13:38:11 2021

@author: shadi
"""



from Node import Node
from ASTAR import getNeighbours

start_node = Node(None,None)

# IDS algorithm
def ids_search(maximum_depth , dim , startPoint , goalPoint , matrix):
    global start_node

    # Create a start node and an goal node
   # start_node = Node(startPoint, (None))
   # goal_node = Node(goalPoint, None)
    
    #Loop for d from 1 to infinity
    for d in range(maximum_depth): 
        visited = initDict(dim)
        #print(d)
        if dls_search(visited,dim,startPoint, goalPoint, matrix,d): 
            return True
    return False




def dls_search(visited,dim , currentNode , goalNode , matrix,limit):
     global start_node
     
     opened = []
   #  closed = []
     depth = 0

     opened.append(currentNode)
     visited[currentNode] = True
      
    # print(currentPoint)
    # print(goalPoint)
  
     # If reached the maximum depth, stop recursing. 
  
     while len(opened) > 0: 
         #print(depth)
         if depth <= limit:
            current = opened.pop(0) 
            
            # visited[current] = False
            if current == goalNode:
                print("Goal Node Found")
                return True
            else: 
                # Get neighbours
                neighbourPoints = getNeighbours(dim,current,matrix)
                for  neighbour in neighbourPoints:
                    # try to visit a node in future, if not already been to it
                    if(not(visited[neighbour])):
                        
                        opened.append(neighbour)
                        visited[neighbour] = True
                depth += 1            
         else:
            print("Not found within depth limit",depth)
            return False


     return False
    
    
    
# Initilizing dictionary for visited nodes
def initDict(dim):

    myDict = dict()

    for i in range(0,dim):
        for j in range(0,dim):
            myDict[(i,j)] = False

    return myDict
    
    
    
    
    
    

    
    