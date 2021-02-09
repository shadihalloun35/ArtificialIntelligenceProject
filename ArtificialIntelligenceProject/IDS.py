# -*- coding: utf-8 -*-
"""
Created on Sat Jan  9 13:38:11 2021

@author: shadi
"""



from Node import Node
from ASTAR import getNeighbours
import sys
import timeit

start_node = Node(None,None)

# IDS algorithm
def ids_search(maximum_depth , dim , startPoint , goalPoint , matrix,runningTimeAllowed,start):
    global start_node
    global path 
    global trackingpath
    global cost 
    global expandedNodes 
    global scannedNodes 
    global PenetrationRatio 
    global minimumDepth 
    global averageDepth 
    global maximumDepth 
    
    # If there is no parent
    NoneParent = Node((-1,-1),(-1,-1))
     
    # Create a start node and an goal node
    start_node = Node(startPoint, NoneParent)
    goal_node = Node(goalPoint, NoneParent)
    
    #Loop for d from 1 to infinity
    for d in range(maximum_depth): 
        
        # Checking if the time has excceded
        if timeit.default_timer() - start > runningTimeAllowed:
            return -2,expandedNodes,-1,-1,scannedNodes,PenetrationRatio,minimumDepth,averageDepth,maximumDepth
        
        visited = initDict(dim)
        if dls_search(visited,dim,start_node, goal_node, matrix,d): 
            return path[::-1],expandedNodes,trackingpath[::-1],cost,scannedNodes,PenetrationRatio,minimumDepth,averageDepth,maximumDepth
    
    return False


def dls_search(visited,dim , currentNode , goalNode , matrix,limit):
     global start_node 
     global path 
     global trackingpath
     global cost 
     global expandedNodes 
     global scannedNodes 
     global PenetrationRatio 
     global minimumDepth 
     global averageDepth 
     global maximumDepth 
     
     opened = []
     cost = 0

     # Variable for max depth
     maximumDepth = 0
     
     # Number of the scanned nodes
     scannedNodes = 0
     
     # Variable for the sum of the heuristic values of the nodes that have been expandeds
     SumHeuristicValues = 0
    
     # Effective Branching Factor
     ebf = 1
     
     # Variable for max depth
     maximumDepth = 0
    
     # Variable for min depth
     minimumDepth = sys.maxsize
    
     # Variable for average depth
     averageDepth = 0
    
     # Variable for average heuristic values
     averageHeuristicValues = 1
     
     # Variable for the sum of the depth of the nodes that have been expanded
     SumDepth = 0
    
     # Variable for the Penetration Ratio (d/N)
     PenetrationRatio = 1
    
     # Previous Node
     previousNode = Node((-1,-1),None)
    
     opened.append(currentNode)
     scannedNodes += 1
     visited[currentNode.point] = True
      
   
  
     # If reached the maximum depth, stop recursing. 
  
     while len(opened) > 0:      
         
         current = opened.pop(0) 
         
         # Calculating the sum of the depth of the nodes
         SumDepth += current.d
         
         # Updating the maximum depth
         if maximumDepth < current.d:
             maximumDepth = current.d
             
         # Updating the minimum depth
         if currentNode.parent != previousNode:
            if minimumDepth > previousNode.d:
                minimumDepth = previousNode.d
             
         if current.d <= limit:
            if current == goalNode:
                
                # Updating the minimum depth
                if minimumDepth > current.d:
                    minimumDepth = current.d
                    
                # Averange Depth
                averageDepth = SumDepth/expandedNodes
                   
                pathlen = 0
                path = []
                trackingpath = []
                while current != start_node:
                    pathlen -= 1
                    x=-pathlen
                    path.append(str(current.point) + ': ' + str(x))
                    current.g = matrix[current.point[0]][current.point[1]]
                    current.f = current.g
                    cost += current.g
                    trackingpath.append(current.point)
                    current = current.parent
                x += 1    
                start_node.g = matrix[start_node.point[0]][start_node.point[1]]
                start_node.f = start_node.g
                path.append(str(start_node.point) + ': ' + str(x))
                trackingpath.append(start_node.point)

                    # Return reversed path
                #print( path[::-1])
                #print("Goal Node Found")
                
                # Penetration Ratio
                PenetrationRatio = maximumDepth/scannedNodes
                
                # Effective Branching Factor
                ebf = scannedNodes ** (1/current.d)
                
                return True# path[::-1]#,expandedNodes,trackingpath[::-1],cost   
            
            else:             
                # Get neighbours
                neighbourPoints = getNeighbours(dim,current.point,matrix)
                for  neighbour in neighbourPoints: 
                    neighborNode = Node(neighbour, current)                   
                    # try to visit a node in future, if not already been to it
                    if(not(visited[neighbour])):
                        expandedNodes += 1
                        neighborNode.d = current.d + 1 
                        
                        opened.append(neighborNode)
                        scannedNodes += 1
                        visited[neighbour] = True
                           
         else:
            #print("Not found within depth limit",current.d)
            return False
      
     # Averange Depth
     averageDepth = SumDepth/expandedNodes
     
     # Averange Heuristic Values
     averageHeuristicValues = SumHeuristicValues/scannedNodes
            
     # Penetration Ratio
     PenetrationRatio = maximumDepth/scannedNodes
     
     # Effective Branching Factor
     ebf = scannedNodes ** (1/current.d)
            
     return False
    
    
    
# Initilizing dictionary for visited nodes
def initDict(dim):

    myDict = dict()

    for i in range(0,dim):
        for j in range(0,dim):
            myDict[(i,j)] = False

    return myDict
    

path = []
trackingpath = []
cost = 0
expandedNodes = 0
scannedNodes = 0
PenetrationRatio = 1 
minimumDepth  = 0
averageDepth = 0
maximumDepth = 0
    
    
    

    
    