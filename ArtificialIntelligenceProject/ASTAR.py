# -*- coding: utf-8 -*-
"""
Created on Thu Jan  7 00:07:56 2021

@author: shadi
"""


from Node import Node

# A* algorithm
def astar_search(dim , startPoint , goalPoint , matrix, heuristicMatrix):
   
    # Counter for the expanded nodes
    expandedNodes = 0
    
    # Create lists for open nodes and closed nodes
    opened = []
    closed = []
    
    # Create a start node and an goal node
    start_node = Node(startPoint, (None))
    goal_node = Node(goalPoint, None)
    
    # Calculate start_node cost
    start_node.g = 0
    start_node.h = heuristicMatrix[startPoint[0]][startPoint[1]]
    start_node.f = start_node.g + start_node.h
     
    # Add the start node
    opened.append(start_node)
    
    # Loop until the open list is empty
    while len(opened) > 0:
        
        # Sort the open list to get the node with the lowest cost first
        opened.sort()
        
        # Get the node with the lowest cost
        current_node = opened.pop(0)
        
        # Add the current node to the closed list
        closed.append(current_node)
        expandedNodes += 1
        
        # Check if we have reached the goal, return the path
        if current_node == goal_node:
        
            path = []
            while current_node != start_node:
                path.append(str(current_node.point) + ': ' + str(current_node.g))
                current_node = current_node.parent
            path.append(str(start_node.point) + ': ' + str(start_node.g))
            
            # Return reversed path
            return path[::-1],expandedNodes
        
        # Get neighbours
        neighborPoints = getNeighbours(dim,current_node.point,matrix)
        
        
        # Loop neighbors
        for myPoint in neighborPoints:
            # Create a neighbor node
            neighborNode = Node(myPoint, current_node)
            
            # Calculate full path cost
            neighborNode.g = current_node.g + matrix[myPoint[0]][myPoint[1]]
            neighborNode.h = heuristicMatrix[myPoint[0]][myPoint[1]]
            neighborNode.f = neighborNode.g + neighborNode.h
            
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
                opened.append(neighborNode)
                
    # Return None, no path is found
    return None

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





















