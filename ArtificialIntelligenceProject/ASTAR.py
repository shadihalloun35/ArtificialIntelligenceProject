# -*- coding: utf-8 -*-
"""
Created on Thu Jan  7 00:07:56 2021

@author: Shadi and Noor
"""


from Node import Node
import heapq 
import sys
import timeit

# A* algorithm
def astar_search(dim , startPoint , goalPoint , matrix, heuristicMatrix,runningTimeAllowed,start):
    
    # If there is no parent
    NoneParent = Node((-1,-1),(-1,-1))
    
    # Number of the expanded nodes
    expandedNodes = 0
    
    # Number of the scanned nodes
    scannedNodes = 0
    
    # Variable for max depth
    maximumDepth = 0
    
    # Effective Branching Factor
    ebf = 1
            
    # Variable for min depth
    minimumDepth = sys.maxsize
    
    # Variable for average depth
    averageDepth = 0
    
    # Variable for average heuristic values
    averageHeuristicValues = 1
    
    # Variable for the sum of the depth of the nodes that have been expanded
    SumDepth = 0
    
    # Variable for the sum of the heuristic values of the nodes that have been expandeds
    SumHeuristicValues = 0
    
    # Variable for the Penetration Ratio (d/N)
    PenetrationRatio = 1
    
    # Previous Node
    previousNode = Node((-1,-1),None)
    
    # Create lists for open nodes and closed nodes
    opened = []
    closed = []
    
    # using heapify() to convert list into heap 
    heapq.heapify(opened) 
    
    # Create a start node and an goal node
    start_node = Node(startPoint, NoneParent)
    goal_node = Node(goalPoint, NoneParent)
    
    # Calculate start_node cost
    start_node.g = 0
    start_node.h = heuristicMatrix[startPoint[0]][startPoint[1]]
    SumHeuristicValues += start_node.h
    start_node.f = start_node.g + start_node.h
    
    # Add the start node
    # Using heappush() to push elements into heap 
    heapq.heappush(opened,start_node) 
    scannedNodes += 1
    
    # Loop until the open list is empty
    while len(opened) > 0:
        
        # Checking if the time has excceded
        if timeit.default_timer() - start > runningTimeAllowed:
            return -2,expandedNodes,-1,-1,scannedNodes,PenetrationRatio,minimumDepth,averageDepth,maximumDepth

        # Get the node with the lowest f cost
        current_node = heapq.heappop(opened)
        
        # Updating the maximum depth
        if maximumDepth < current_node.d:
            maximumDepth = current_node.d
          
        # Updating the minimum depth
        if current_node.parent != previousNode:
            if minimumDepth > previousNode.d:
                minimumDepth = previousNode.d
                    
        # Saving the previous node
        previousNode = current_node
        
        # Add the current node to the closed list
        closed.append(current_node)
        
        # Calculating the sum of the depth of the nodes
        SumDepth += current_node.d
    
        # This node have been expanded
        expandedNodes += 1
        
        # Check if we have reached the goal, return the path
        if current_node == goal_node:
        
            # Updating the minimum depth
            if minimumDepth > current_node.d:
                minimumDepth = current_node.d
                
            path = []
            trackingpath = []
            cost = current_node.g
            
            while current_node != start_node:
                path.append(str(current_node.point) + ': ' + str(current_node.g))
                trackingpath.append(current_node.point)
                current_node = current_node.parent
            path.append(str(start_node.point) + ': ' + str(start_node.g))
            trackingpath.append(start_node.point)

            # Averange Depth
            averageDepth = SumDepth/expandedNodes
            
            # Averange Heuristic Values
            averageHeuristicValues = SumHeuristicValues/scannedNodes
            
            # Penetration Ratio
            PenetrationRatio = maximumDepth/scannedNodes
            
            # Effective Branching Factor
            ebf = scannedNodes ** (1/current_node.d)
            
            # Return reversed path
            return path[::-1],expandedNodes,trackingpath[::-1],cost,scannedNodes,PenetrationRatio,minimumDepth,averageDepth,maximumDepth
        
        # Get neighbours
        neighborPoints = getNeighbours(dim,current_node.point,matrix)
        
        # Loop neighbors
        for myPoint in neighborPoints:
            # Create a neighbor node
            neighborNode = Node(myPoint, current_node)
            
            # Calculate full path cost
            neighborNode.g = current_node.g + matrix[myPoint[0]][myPoint[1]]
            neighborNode.h = heuristicMatrix[myPoint[0]][myPoint[1]]
            SumHeuristicValues += neighborNode.h
            neighborNode.f = neighborNode.g + neighborNode.h
            neighborNode.d = current_node.d + 1
            
                
            # Check if the neighbor is in the closed list
            if(neighborNode in closed):
               # if remove_from_closed(closed,neighborNode):
                #    closed.remove(neighborNode)
                 #   opened.append(neighborNode)
                continue

            # Check if neighbor is in open list and if it has a lower f value
            if(add_to_open(opened, neighborNode) == True):
                
                #Substitute it for the old value
                if neighborNode in opened:
                    opened.remove(neighborNode)
                    
                # Add neighbor to open list
                heapq.heappush(opened,neighborNode) 
                scannedNodes += 1

    # Averange Depth
    averageDepth = SumDepth/expandedNodes
            
    # Averange Heuristic Values
    averageHeuristicValues = SumHeuristicValues/scannedNodes
            
    # Penetration Ratio
    PenetrationRatio = maximumDepth/scannedNodes
            
    # Effective Branching Factor
    ebf = scannedNodes ** (1/current_node.d)
            
    # Return None, no path is found
    return -1,expandedNodes,-1,-1,scannedNodes,PenetrationRatio,minimumDepth,averageDepth,maximumDepth

# Check if a neighbor should be added to open list
def add_to_open(opened, neighbor):
    for node in opened:
        if (neighbor == node and neighbor.f > node.f):
            return False
    return True

# Check if a neighbor should be removed from closed list
def remove_from_closed(closed,neighbor):
    for node in closed:
        if (neighbor == node and neighbor.f > node.f):
            return True
    return False

# Finding the neighbours of the current point
def getNeighbours(dim,currentNodePoint,matrix):

    myList = []
    for i in range(currentNodePoint[0]-1, currentNodePoint[0]+2):
        for j in range(currentNodePoint[1]-1, currentNodePoint[1]+2):
            if i >= 0 and i < dim and j >= 0 and j < dim and matrix[i][j] != -1 and (i != currentNodePoint[0] or j != currentNodePoint[1]):
                myList.append((i,j))  

    return myList     





















