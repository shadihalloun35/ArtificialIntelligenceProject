# -*- coding: utf-8 -*-
"""
Created on Fri Jan 15 20:15:48 2021

@author: shadi
"""


from Node import Node
from ASTAR import getNeighbours
from ASTAR import add_to_open

# IDA* algorithm
def idastar_search(maximum_depth , dim , startPoint , goalPoint , matrix, heuristicMatrix):
    
    global start_node

    threshold = heuristicMatrix[startPoint[0]][startPoint[1]]

    # Create a start node and an goal node
    start_node = Node(startPoint, (None))
    goal_node = Node(goalPoint, None)
    
    
    #Loop for infinity
    while(1): 
        
        #visited = initDict(dim)
        
        distance,found = dfsContour(visited,dim,start_node, goal_node, matrix,heuristicMatrix,0,threshold)
        
        #print(distance)
        #print(found)

        if found == True:
            print('solotuion')                             
            return True
        
        else: 
            threshold = distance  
    
    
def dfsContour(visited,dim , startNode , goalNode , matrix,heuristicMatrix,distance,threshold):
     global start_node
     opened = []
     
     opened.append(startNode)
     
     startNode.f = distance + heuristicMatrix[startNode.point[0]][startNode.point[1]]

     if startNode.f > threshold:
         #print("Breached threshold with heuristic: " + str(f))
         return f,False
     
     if startNode == goalNode:
         
        # We have found the goal node we we're searching for
        print(str(startNode.point) + ': ' + str(startNode.g))
        return startNode,True
   
    
     minVal = float("inf")
     
     while len(opened) != 0:
         #print(len(opened))
         currentNode = opened.pop(0) 
         #print(len(opened))
         if currentNode.f <= threshold:
             
             if currentNode == goalNode:
                    # We have found the goal node we we're searching for
                path = []
                while currentNode != startNode:
                     path.append(str(currentNode.point) + ': ' + str(currentNode.g))
                     currentNode = currentNode.parent
                path.append(str(startNode.point) + ': ' + str(startNode.g))
                    
                # Return reversed path
                print(path[::-1])
                
                print("Goal Node Found")
                return path[::-1],True         
                          
             minVal = float("inf")
             neighbourPoints = getNeighbours(dim,currentNode.point,matrix)
    
             for neighbour in neighbourPoints:
                  
                 neighborNode = Node(neighbour, currentNode)                   
                 # try to visit a node in future, if not already been to it
               
                 neighborNode.g = currentNode.g + matrix[neighborNode.point[0]][neighborNode.point[1]]
                 neighborNode.f = neighborNode.g + heuristicMatrix[neighborNode.point[0]][neighborNode.point[1]]
                 if(add_to_open(opened, neighborNode) == True):
                
                    #Substitute it for the old value
                    if neighborNode in opened:
                        opened.remove(neighborNode)
                        
                    # Add neighbor to open list
                    opened.append(neighborNode)
                 #opened.append(neighborNode)
                 
         else:
            
            if currentNode.f < minVal:
                minVal = currentNode.f
          
    # print('minval : ',minVal)            
     return minVal,False
             
             

# Initilizing dictionary for visited nodes
def initDict(dim):

    myDict = dict()

    for i in range(0,dim):
        for j in range(0,dim):
            myDict[(i,j)] = False

    return myDict















