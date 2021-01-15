# -*- coding: utf-8 -*-
"""
Created on Fri Jan 15 20:15:48 2021

@author: shadi
"""


from Node import Node
from ASTAR import getNeighbours

# IDA* algorithm
def idastar_search(maximum_depth , dim , startPoint , goalPoint , matrix, heuristicMatrix):
    
    global start_node

    threshold = heuristicMatrix[startPoint[0]][startPoint[1]]

    # Create a start node and an goal node
    start_node = Node(startPoint, (None))
    goal_node = Node(goalPoint, None)
    
    #Loop for infinity
    while(1): 
        visited = initDict(dim)
        
        distance  = dfsContour(visited,dim,start_node, goal_node, matrix,heuristicMatrix,0,threshold)
        if distance  == maximum_depth:
             return -1;                                           
        elif distance  < 0:                               
            return -distance
        
        else: 
            threshold = distance  
    
    
def dfsContour(visited,dim , currentNode , goalNode , matrix,heuristicMatrix,distance,threshold):
     global start_node
     
     opened = []
     
     opened.append(currentNode)
     visited[currentNode.point] = True
     
     if currentNode == goalNode:
         
        # We have found the goal node we we're searching for
        return -distance
   
     f = distance + heuristicMatrix[currentNode.point[0]][currentNode.point[1]]
     if f > threshold:
         
         print("Breached threshold with heuristic: " + str(f))
         return f
     
     min = float("inf")
     neighbourPoints = getNeighbours(dim,currentNode.point,matrix)
     
     for neighbour in neighbourPoints: 
         neighborNode = Node(neighbour, currentNode)                   
         # try to visit a node in future, if not already been to it
         if(not(visited[neighbour])):
             
             t = dfsContour(visited,dim , neighborNode , goalNode , matrix,heuristicMatrix,distance + matrix[neighborNode.point[0]][neighborNode.point[1]] ,threshold)
             
             if t < 0:
                # Node found
                return t
             elif t < min:
                min = t
             opened.append(neighborNode)
             visited[neighbour] = True
             
     return min

             
             
             
             
'''
     for i in range(len(tree[node])):
        if tree[node][i] != 0:
            t = iterative_deepening_a_star_rec(tree, heuristic, i, goal, distance + tree[node][i], threshold)
            if t < 0:
                # Node found
                return t
            elif t < min:
                min = t
                
     # If reached the maximum depth, stop recursing. 
     while len(opened) > 0:      
         current = opened.pop(0) 
         if current.d <= limit:
            if current == goalNode:
                pathlen = 0
                path = []
                while current != start_node:
                    pathlen-=1
                    x=-pathlen
                    path.append(str(current.point) + ': ' + str(x))
                    current = current.parent
                x+=1    
                path.append(str(start_node.point) + ': ' + str(x))
                    
                    # Return reversed path
                print( path[::-1])
                
                print("Goal Node Found")
                return True      
            else:             
                # Get neighbours
                neighbourPoints = getNeighbours(dim,current.point,matrix)
                for  neighbour in neighbourPoints: 
                    neighborNode = Node(neighbour, current)                   
                    # try to visit a node in future, if not already been to it
                    if(not(visited[neighbour])):
                        neighborNode.d = current.d + 1 
                        opened.append(neighborNode)
                        visited[neighbour] = True
                           
         else:
            print("Not found within depth limit",current.d)
            return False
        
     return False
'''
















# Initilizing dictionary for visited nodes
def initDict(dim):

    myDict = dict()

    for i in range(0,dim):
        for j in range(0,dim):
            myDict[(i,j)] = False

    return myDict