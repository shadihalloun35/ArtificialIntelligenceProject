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

        distance,found = dfsContour(dim,start_node, goal_node, matrix,heuristicMatrix,0,threshold)
        
        #print(distance)
        #print(found)

        if found == True:
            print('solotuion')                             
            return True
        
        else: 
            threshold = distance  
    
    
def dfsContour(dim , startNode , goalNode , matrix,heuristicMatrix,distance,threshold):
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































'''
# IDA* algorithm
def idastar_search(maximum_depth , dim , startPoint , goalPoint , matrix, heuristicMatrix):
    
    threshold = heuristicMatrix[startPoint[0]][startPoint[1]]
    
    # Create a start node and an goal node
    start_node = Node(startPoint, (None))
    goal_node = Node(goalPoint, None)
    
    mySolotuion = initDict(dim)
    #Loop for infinity
    while(1): 
            
        distance  = dfsContour(mySolotuion,dim,start_node, goal_node, matrix,heuristicMatrix,0,threshold)
       
        print(distance)
        if distance  == float("inf"):
             return -1;     
                                      
        elif distance  < 0:  
            print('solotuion')                             
            return -distance
       
        else: 
            threshold = distance  
    
    
def dfsContour(mySolotuion,dim , currentNode , goalNode , matrix,heuristicMatrix,distance,threshold):
     
     
     
     if currentNode == goalNode:
        # We have found the goal node we we're searching for
        return -distance
    
    
     f = distance + heuristicMatrix[currentNode.point[0]][currentNode.point[1]]
     
     #mySolotuion[currentNode.point] = f
     
     currentNode.g = distance
     
     if f > currentNode.f:
          return threshold
      
     else:          
          currentNode.f = f
     
        
     if f > threshold:
         return f
     
     min_f = float("inf")
     
     neighbourPoints = getNeighbours(dim,currentNode.point,matrix)

     
     while (1):
         
         for neighbour in neighbourPoints:
             
             neighborNode = Node(neighbour, currentNode)                   
             neighborNode.g = currentNode.g + matrix[neighborNode.point[0]][neighborNode.point[1]] 
             
             currentNode = neighborNode
             
             f = currentNode.g + heuristicMatrix[currentNode.point[0]][currentNode.point[1]]
             
             if f > currentNode.f:
                 t = threshold
      
             else:          
                 currentNode.f = f
             
             if f > threshold:
                 t = f
                 
                 if t < 0:
                     return t
                   
                 elif t < min_f:
                     min_f = t
                     
             elif currentNode == goalNode:
                 # We have found the goal node we we're searching for
                 return -currentNode.g
              
             else:
                 min_f = float("inf")
                 neighbourPoints = getNeighbours(dim,currentNode.point,matrix)
                 break
                                                      
         t = min_f
         print(t)
    
                 
             
             
             

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



# Initilizing dictionary for visited nodes
def initDict(dim):

    myDict = dict()

    for i in range(0,dim):
        for j in range(0,dim):
            myDict[(i,j)] = 0

    return myDict

'''