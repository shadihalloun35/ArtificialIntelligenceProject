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
    global path 
    global trackingpath
    global cost 
    global expandedNodes 
    
    # Create a start node and an goal node
    start_node = Node(startPoint, None)
    goal_node = Node(goalPoint, None)
    
    #Loop for d from 1 to infinity
    for d in range(maximum_depth): 
        visited = initDict(dim)
        if dls_search(visited,dim,start_node, goal_node, matrix,d): 
            return path[::-1],expandedNodes,trackingpath[::-1],cost 
    
    return False


def dls_search(visited,dim , currentNode , goalNode , matrix,limit):
     global start_node 
     global path 
     global trackingpath
     global cost 
     global expandedNodes 
     opened = []
     cost = 0

     opened.append(currentNode)
     visited[currentNode.point] = True
      
   
  
     # If reached the maximum depth, stop recursing. 
  
     while len(opened) > 0:      
         current = opened.pop(0) 
         if current.d <= limit:
            if current == goalNode:
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
                        visited[neighbour] = True
                           
         else:
            #print("Not found within depth limit",current.d)
            return False
        
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

    
    
    

    
    