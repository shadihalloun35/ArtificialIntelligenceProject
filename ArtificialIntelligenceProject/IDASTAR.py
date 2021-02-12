# -*- coding: utf-8 -*-
"""
Created on Fri Jan 15 20:15:48 2021

@author: shadi
"""


from Node import Node
from ASTAR import getNeighbours
from ASTAR import add_to_open
import sys
import timeit

# IDA* algorithm
def idastar_search(maximum_depth , dim , startPoint , goalPoint , matrix, heuristicMatrix,runningTimeAllowed,start):
    
    global start_node

    # If there is no parent
    NoneParent = Node((-1,-1),(-1,-1))
     
    threshold = heuristicMatrix[startPoint[0]][startPoint[1]]

    # Create a start node and an goal node
    start_node = Node(startPoint, NoneParent)
    goal_node = Node(goalPoint, NoneParent)
    
    
    #Loop for infinity
    while(1): 
        
        # Calling the contour function with a particular threshold
        trackingpath,cost,expandedNodes,distance,scannedNodes,PenetrationRatio,minimumDepth,averageDepth,maximumDepth,averageHeuristicValues,ebf,PenetrationY,found = dfsContour(dim,start_node, goal_node, matrix,heuristicMatrix,0,threshold,runningTimeAllowed,start)
        
         # Checking if the time has excceded
        if timeit.default_timer() - start > runningTimeAllowed:
            return -2,cost,expandedNodes,distance,scannedNodes,PenetrationRatio,minimumDepth,averageDepth,maximumDepth,averageHeuristicValues,ebf,PenetrationY
        
        # Path was found
        if found == True:
            return distance,expandedNodes,trackingpath[::-1],cost,scannedNodes,PenetrationRatio,minimumDepth,averageDepth,maximumDepth,averageHeuristicValues,ebf,PenetrationY
        
        #elif distance ==  float("inf"):
     #       return -1,-1,-1,-1
        
        else: 
            # Calling the function with a bigger threshold
            threshold = distance  
    
    
def dfsContour(dim , startNode , goalNode , matrix,heuristicMatrix,distance,threshold,runningTimeAllowed,start):
     global start_node
     
     # Opened List
     opened = []
     
     # Variable for max depth
     maximumDepth = 0
    
     # Effective Branching Factor
     ebf = 1
               
     # Success (Y/N)
     PenetrationY = 1
    
     # Number of the expanded nodes
     expandedNodes = 0
     
     # Variable for the sum of the heuristic values of the nodes that have been expandeds
     SumHeuristicValues = 0
    
     # Number of the scanned nodes
     scannedNodes = 0
    
     # Variable for max depth
     maximumDepth = 0
    
     # Variable for min depth
     minimumDepth = sys.maxsize
    
     # Variable for average heuristic values
     averageHeuristicValues = 1
     
     # Variable for average depth
     averageDepth = 0
    
     # Variable for the sum of the depth of the nodes that have been expanded
     SumDepth = 0
    
     # Variable for the Penetration Ratio (d/N)
     PenetrationRatio = 1
    
     # Previous Node
     previousNode = Node((-1,-1),None)
    
     # Inserting the first node to the open list
     opened.append(startNode)
     scannedNodes += 1

     # Calcilating the f cost of the first node
     startNode.f = distance + heuristicMatrix[startNode.point[0]][startNode.point[1]]
     startNode.h = distance
     SumHeuristicValues += distance

     # There is no f cost higher than the threeshold so we return 
     # the f cost to make our first iteration
     if startNode.f > threshold:
         return -1,-1,-1,startNode.f,scannedNodes,PenetrationRatio,minimumDepth,averageDepth,maximumDepth,averageHeuristicValues,ebf,PenetrationY,False
     
     # Check if we have reached the goal, return the path
     if startNode == goalNode:
         
        cost = startNode.g
        # We have found the goal node we we're searching for
        return [startNode],cost,1,[str(startNode.point) + ': ' + str(startNode.g)],scannedNodes,PenetrationRatio,minimumDepth,averageDepth,maximumDepth,averageHeuristicValues,ebf,PenetrationY,True
   
    
     minVal = float("inf")
     
     cost = startNode.g
     
     # Loop until there is 
     while len(opened) != 0:
         
        currentNode = opened.pop(0) 
        
        # Calculating the sum of the depth of the nodes
        SumDepth += currentNode.d
        
        # Updating the maximum depth
        if maximumDepth < currentNode.d:
            maximumDepth = currentNode.d
                     
             
        # Updating the minimum depth
        if currentNode.parent != previousNode:
            if minimumDepth > previousNode.d:
                minimumDepth = previousNode.d
                    
        # Saving the previous node
        previousNode = currentNode
        
        if currentNode.f <= threshold:
            expandedNodes += 1
            
            # Check if we have reached the goal, return the path
            # We have found the goal node we we're searching for
            if currentNode == goalNode:
                
               # Updating the minimum depth
               if minimumDepth > currentNode.d:
                   minimumDepth = currentNode.d
               
               # Averange Depth
               averageDepth = SumDepth/expandedNodes
               
               # Averange Heuristic Values
               averageHeuristicValues = SumHeuristicValues/scannedNodes
               
               path = []
               trackingpath = []
               cost = currentNode.g
               
               # Effective Branching Factor
               ebf = scannedNodes ** (1/currentNode.d)
     
               # Success
               PenetrationY = currentNode.d/expandedNodes
               
               while currentNode != startNode:
                    path.append(str(currentNode.point) + ': ' + str(currentNode.g))
                    trackingpath.append(currentNode.point)
                    currentNode = currentNode.parent
                    
               path.append(str(startNode.point) + ': ' + str(startNode.g))
               trackingpath.append(startNode.point)
               
               # Penetration Ratio
               PenetrationRatio = maximumDepth/scannedNodes
               
               # Return reversed path
               return trackingpath,cost,expandedNodes,path[::-1],scannedNodes,PenetrationRatio,minimumDepth,averageDepth,maximumDepth,averageHeuristicValues,ebf,PenetrationY,True         
                         
            minVal = float("inf")
            neighbourPoints = getNeighbours(dim,currentNode.point,matrix)
           
            for neighbour in neighbourPoints:
                 
                neighborNode = Node(neighbour, currentNode)                   
                # try to visit a node in future, if not already been to it
              
                neighborNode.g = currentNode.g + matrix[neighborNode.point[0]][neighborNode.point[1]]
                neighborNode.h = heuristicMatrix[neighborNode.point[0]][neighborNode.point[1]]
                neighborNode.f = neighborNode.g + neighborNode.h
                SumHeuristicValues += neighborNode.h
                neighborNode.d = currentNode.d + 1
                
              
                if(add_to_open(opened, neighborNode) == True):
               
                   #Substitute it for the old value
                   if neighborNode in opened:
                       opened.remove(neighborNode)
                       
                   # Add neighbor to open list
                   opened.append(neighborNode)
                   scannedNodes += 1
                
        else:
           
           if currentNode.f > threshold and currentNode.f < minVal:
               minVal = currentNode.f
           
     # Averange Depth
     averageDepth = SumDepth/expandedNodes
     
     # Averange Heuristic Values
     averageHeuristicValues = SumHeuristicValues/scannedNodes
     
     # Effective Branching Factor
     ebf = scannedNodes ** (1/currentNode.d)
    
     # Penetration Ratio
     PenetrationRatio = maximumDepth/scannedNodes
     
     # Success
     PenetrationY = currentNode.d/expandedNodes
               
     return -1,-1,-1,minVal,scannedNodes,PenetrationRatio,minimumDepth,averageDepth,maximumDepth,averageHeuristicValues,ebf,PenetrationY,False
             
             

# Initilizing dictionary for visited nodes
def initDict(dim):

    myDict = dict()

    for i in range(0,dim):
        for j in range(0,dim):
            myDict[(i,j)] = False

    return myDict















