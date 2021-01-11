# -*- coding: utf-8 -*-
"""
Created on Mon Jan 11 19:02:24 2021

@author: Shadi and Noor
"""


from Node import Node

# Bidirectional A* algorithm
def biastar_search(dim , startPoint , goalPoint , matrix, heuristicMatrix):
   
    # Counter for the expanded nodes
    expandedNodes = 0
    
    # Create lists for open nodes and closed nodes for forward direction 
    forwardOpened = []
    forwardClosed = []
    
    # Create lists for open nodes and closed nodes for backward direction  
    backwardOpened = []
    backwardClosed = []
    
    
    start_node = Node(startPoint, (None))
    goal_node = Node(goalPoint, None)
    
    # Calculate start_node cost
    start_node.g = 0
    start_node.h = heuristicMatrix[startPoint[0]][startPoint[1]]
    start_node.f = start_node.g + start_node.h
    
    goal_node.g = 0
    goal_node.h = heuristicMatrix[goalPoint[0]][goalPoint[1]]
    goal_node.f = goal_node.g + goal_node.h
     
    # Add the start node
    forwardOpened.append(start_node) 
    backwardOpened.append(goal_node)
    
    # Loop until the open list is empty
    while len(forwardOpened) > 0 and len(backwardOpened) > 0:
        
        # Sort the open list to get the node with the lowest cost first
        forwardOpened.sort()
        backwardOpened.sort()

        # Get the node with the lowest cost
        current_node_forward = forwardOpened.pop(0)
        current_node_backward = backwardOpened.pop(0)

        # Add the current node to the closed list
        forwardClosed.append(current_node_forward)
        backwardClosed.append(current_node_backward)

        expandedNodes += 2
        
        # Check if we have reached the goal, return the path
        if current_node_forward in backwardClosed or current_node_backward in forwardClosed :
            path = []
            while current_node != start_node:
                path.append(str(current_node.point) + ': ' + str(current_node.g))
                current_node = current_node.parent
            path.append(str(start_node.point) + ': ' + str(start_node.g))
            
            # Return reversed path
            return path[::-1],expandedNodes
        
        
        # Get neighbours
        neighborPointsForward = getNeighbours(dim,current_node_forward.point,matrix)
        neighborPointsBackward = getNeighbours(dim,current_node_backward.point,matrix)

        # Loop neighbors forward
        for myPointForward in neighborPointsForward:
            # Create a neighbor node
            neighborNodeForward = Node(myPointForward, current_node_forward)
            
            # Calculate full path cost
            neighborNodeForward.g = current_node_forward.g + matrix[myPointForward[0]][myPointForward[1]]
            neighborNodeForward.h = heuristicMatrix[myPointForward[0]][myPointForward[1]]
            neighborNodeForward.f = neighborNodeForward.g + neighborNodeForward.h
            
            # Check if the neighbor is in the closed list
            if(neighborNodeForward in forwardClosed):
               # if remove_from_closed(closed,neighborNode):
                #    closed.remove(neighborNode)
                 #   opened.append(neighborNode)
                continue
            
            # Check if neighbor is in open list and if it has a lower f value
            if(add_to_open(forwardOpened, neighborNodeForward) == True):
                
                #Substitute it for the old value
                if neighborNodeForward in forwardOpened:
                    forwardOpened.remove(neighborNodeForward)
                    
                # Add neighbor to open list
                forwardOpened.append(neighborNodeForward)
          
        for myPointBackward in neighborPointsBackward:
            # Create a neighbor node
            neighborNodeBackward = Node(myPointBackward, current_node_backward)
            
            # Calculate full path cost
            neighborNodeForward.g = current_node_forward.g + matrix[myPointForward[0]][myPointForward[1]]
            neighborNodeForward.h = heuristicMatrix[myPointForward[0]][myPointForward[1]]
            neighborNodeForward.f = neighborNodeForward.g + neighborNodeForward.h
            
            # Check if the neighbor is in the closed list
            if(neighborNodeBackward in backwardClosed):
               # if remove_from_closed(closed,neighborNode):
                #    closed.remove(neighborNode)
                 #   opened.append(neighborNode)
                continue
            
            # Check if neighbor is in open list and if it has a lower f value
            if(add_to_open(backwardOpened, neighborNodeBackward) == True):
                
                #Substitute it for the old value
                if neighborNodeBackward in backwardOpened:
                    backwardOpened.remove(neighborNodeBackward)
                    
                # Add neighbor to open list
                backwardOpened.append(neighborNodeBackward)
                
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

    

    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    