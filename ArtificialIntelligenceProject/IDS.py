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
    start_node = Node(startPoint, (None))
    goal_node = Node(goalPoint, None)
    
    #Loop for d from 1 to infinity
    for d in range(maximum_depth): 
        visited = initDict(dim)
        if dls_search(visited,dim,start_node, goal_node, matrix,d): 
            return True
    
    return False


def dls_search(visited,dim , currentNode , goalNode , matrix,limit):
     global start_node
     
     opened = []
     

     opened.append(currentNode)
     visited[currentNode.point] = True
      
   
  
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
    
    
    
# Initilizing dictionary for visited nodes
def initDict(dim):

    myDict = dict()

    for i in range(0,dim):
        for j in range(0,dim):
            myDict[(i,j)] = False

    return myDict
    
    
    
    
    
    

    
    