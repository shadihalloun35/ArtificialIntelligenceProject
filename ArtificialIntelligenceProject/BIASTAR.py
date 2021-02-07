# -*- coding: utf-8 -*-
"""
Created on Mon Jan 11 19:02:24 2021
@author: Shadi and Noor
"""


from Node import Node
from ASTAR import getNeighbours
from ASTAR import add_to_open
import sys

# Bidirectional A* algorithm
def biastar_search(dim , startPoint , goalPoint , matrix, forwardHeuristicMatrix,backwardHeuristicMatrix):
   
    # Counter for the expanded nodes
    expandedNodes = 0
    
    # Create lists for open nodes and closed nodes for forward direction 
    forwardOpened = []
    forwardClosed = []
    
    # Create lists for open nodes and closed nodes for backward direction  
    backwardOpened = []
    backwardClosed = []
    
    
    start_node = Node(startPoint, None)
    goal_node = Node(goalPoint, None)
    
    # Calculate start_node cost foward way
    start_node.g = 0
    start_node.h = forwardHeuristicMatrix[startPoint[0]][startPoint[1]]
    start_node.f = start_node.g + start_node.h
    
    # Calculate start_node cost backward way
    goal_node.g = 0
    goal_node.h = backwardHeuristicMatrix[goalPoint[0]][goalPoint[1]]
    goal_node.f = goal_node.g + goal_node.h

    # Add the start node
    forwardOpened.append(start_node) 
    backwardOpened.append(goal_node)
    
    # Loop until the open list is empty
    while len(forwardOpened) > 0 and len(backwardOpened) > 0:  ###### remmember to fix this and when the goal has -1 value in matrix
        
        # Sort the open list to get the node with the lowest cost first
        forwardOpened.sort()
        backwardOpened.sort()

        # Get the node with the lowest cost
        #if len(forwardOpened) > 0:
        current_node_forward = forwardOpened.pop(0)
            
        #if len(backwardOpened) > 0:  
        current_node_backward = backwardOpened.pop(0)

        # Add the current node to the closed list
        #if len(forwardOpened) > 0:
        forwardClosed.append(current_node_forward)
            
        #if len(backwardOpened) > 0:  
        backwardClosed.append(current_node_backward)
        
        expandedNodes += 2
         
        # Get neighbours
        neighborPointsForward = getNeighbours(dim,current_node_forward.point,matrix)
        
        neighborPointsBackward = getNeighbours(dim,current_node_backward.point,matrix)

        # Loop neighbors forward
        for myPointForward in neighborPointsForward:
            # Create a neighbor node
            neighborNodeForward = Node(myPointForward, current_node_forward)
            
            # Calculate full path cost
            neighborNodeForward.g = current_node_forward.g + matrix[myPointForward[0]][myPointForward[1]]
            neighborNodeForward.h = forwardHeuristicMatrix[myPointForward[0]][myPointForward[1]]
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
            neighborNodeBackward.g = current_node_backward.g + matrix[myPointBackward[0]][myPointBackward[1]]
            neighborNodeBackward.h = backwardHeuristicMatrix[myPointBackward[0]][myPointBackward[1]]
            neighborNodeBackward.f = neighborNodeBackward.g + neighborNodeBackward.h
            
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
                
                
        # After Node was found in both closed lists
        if current_node_backward in forwardClosed:
            path,trackingpath,totalSumG = findpath(start_node,goal_node,forwardOpened,forwardClosed,backwardOpened,backwardClosed,matrix,goalPoint)
            return path,expandedNodes,trackingpath,totalSumG
        
        if current_node_forward in backwardClosed :            
            path,trackingpath,totalSumG = findpath(start_node,goal_node,forwardOpened,forwardClosed,backwardOpened,backwardClosed,matrix,goalPoint)
            return path,expandedNodes,trackingpath,totalSumG
        
        
    # Return None, no path is found
    return -1,-1

# Check if a neighbor should be removed from closed list
def remove_from_closed(closed,neighbor):
    for node in closed:
        if (neighbor == node and neighbor.f > node.f):
            return True
    return False
 
def findpath(start_node,goal_node,forwardOpened,forwardClosed,backwardOpened,backwardClosed,matrix,goalPoint):
    pathForward = []
    pathBackward = []
    trackingpathForward = []
    trackingpathBackward = []
    totalSumG = 0
    forawrdNodes = forwardOpened + forwardClosed
    backwardsNodes = backwardOpened + backwardClosed
    minValue = sys.maxsize
    minNodeForward = Node(None,None)
    minNodeBackward = Node(None,None)
    '''
    print('Forward opened:',forwardOpened)
    print('------------------')
    print('backward opend:',backwardOpened)
    print('------------------')
    print('Forwad closed:',forwardClosed)
    print('------------------')
    print('backward closed:',backwardClosed)
    print('------------------')
    '''
    for nodeForward in forawrdNodes:
        for nodeBackward in backwardsNodes:           
            if nodeForward == nodeBackward:
                if (nodeForward.f + nodeBackward.f) < minValue:
                   # print('hhhhhhhhhhhhhhhhhhhhhh')
                    minValue = nodeForward.f + nodeBackward.f
                    minNodeForward = nodeForward
                    minNodeBackward = nodeBackward
                    '''   
    for nodeOpen in backwardOpened:
        for nodeClosed in forwardClosed:
            if nodeOpen == nodeClosed:
                if (nodeOpen.f + nodeClosed.f) < minValue:
                    minValue = nodeOpen.f + nodeClosed.f
                    minNodeOpen = nodeOpen
                    minNodeClosed = nodeClosed
                    '''
   
    savedForwardNode = minNodeForward
    while minNodeForward != start_node:
        
        pathForward.append(str(minNodeForward.point) + ': ' + str(minNodeForward.g))
        trackingpathForward.append(minNodeForward.point)
        minNodeForward = minNodeForward.parent        
        
    pathForward.append(str(start_node.point) + ': ' + str(start_node.g))
    trackingpathForward.append(start_node.point)

    savedBackwardNode = minNodeBackward
    while minNodeBackward != goal_node:
        pathBackward.append(str(minNodeBackward.point) + ': ' + str(minNodeBackward.g))
        trackingpathBackward.append(minNodeBackward.point)
        minNodeBackward = minNodeBackward.parent

    pathBackward.append(str(goal_node.point) + ': ' + str(goal_node.g))
    trackingpathBackward.append(goal_node.point)

    
    if savedForwardNode.g > savedBackwardNode.g:
        pathForward.pop(0)
        trackingpathForward.pop(0)

    else:
        pathBackward.pop(0)
        trackingpathBackward.pop(0)

    
    # Total cost of the path
    totalSumG = int(pathBackward[0].split(':')[1]) + int(pathForward[0].split(':')[1]) + matrix[goalPoint[0]][goalPoint[1]]
    trackingpath = trackingpathForward[::-1] + trackingpathBackward
    # Return reversed path
    #return  pathBackward[::-1] + pathForward,totalSumG
    return  pathForward[::-1] + pathBackward,trackingpath,totalSumG