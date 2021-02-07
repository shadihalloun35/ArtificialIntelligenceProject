# -*- coding: utf-8 -*-
"""
Created on Wed Jan  6 13:24:36 2021
@author: Shadi and Noor
"""

import ASTAR
import BIASTAR
import IDS
import IDASTAR
import math
import sys
import UCS

from HeuristicFunctions import calcEuclideanHeuristic
#from HeuristicFunctions import calcBackwardEuclideanHeuristic

import timeit

start = timeit.default_timer()

#Your statements here

def findTrackingpath(trackingpath):
    trackSet = ''
    startingPoint = trackingpath[0]
    for i in range(len(trackingpath) - 1):
        
        if trackingpath[i][0] < trackingpath[i+1][0] and trackingpath[i][1] < trackingpath[i+1][1]:
            trackSet = trackSet + '-RD'
            
        elif trackingpath[i][0] < trackingpath[i+1][0] and trackingpath[i][1] > trackingpath[i+1][1]:
            trackSet = trackSet + '-LD'

        elif trackingpath[i][0] < trackingpath[i+1][0] and trackingpath[i][1] == trackingpath[i+1][1]:
            trackSet = trackSet + '-D'
            
        elif trackingpath[i][0] > trackingpath[i+1][0] and trackingpath[i][1] > trackingpath[i+1][1]:
            trackSet = trackSet + '-LU'
    
        elif trackingpath[i][0] > trackingpath[i+1][0] and trackingpath[i][1] < trackingpath[i+1][1]:
            trackSet = trackSet + '-RU'
            
        elif trackingpath[i][0] > trackingpath[i+1][0] and trackingpath[i][1] == trackingpath[i+1][1]:
            trackSet = trackSet + '-U'
            
        elif trackingpath[i][0] == trackingpath[i+1][0] and trackingpath[i][1] < trackingpath[i+1][1]:
            trackSet = trackSet + '-R'
    
        elif trackingpath[i][0] == trackingpath[i+1][0] and trackingpath[i][1] > trackingpath[i+1][1]:
            trackSet = trackSet + '-L'
            
    return trackSet[1:]


f = open("firstInput.txt", "r")
savedInput  = f.read().splitlines()
algorithm = savedInput[0]
dim = int(savedInput[1])
startPoint_list = savedInput[2].split(',')
startPoint = (int(startPoint_list[0]),int(startPoint_list[0]))
goalPoint_list = savedInput[3].split(',')
goalPoint = (int(goalPoint_list[0]),int(goalPoint_list[1]))
matrix = [[int(num) for num in line.split(',')] for line in savedInput[4:]]
limitedRunTime = 200
trackingpath = []


if algorithm == 'ASTAR':
    euclideanHeuristicMatrix = calcEuclideanHeuristic(goalPoint,dim,matrix)
    path,expandedNodes,trackingpath,cost = ASTAR.astar_search(dim , startPoint , goalPoint , matrix, euclideanHeuristicMatrix)

elif algorithm == 'UCS':
    path,expandedNodes,trackingpath,cost = UCS.ucs_search(dim , startPoint , goalPoint , matrix)

elif algorithm == 'IDS':
    path,expandedNodes,trackingpath,cost = IDS.ids_search(sys.maxsize , dim , startPoint , goalPoint , matrix)

elif algorithm == 'BIASTAR':
    ForwardEuclideanHeuristicMatrix = calcEuclideanHeuristic(goalPoint,dim,matrix)
    BackwardEuclideanHeuristicMatrix = calcEuclideanHeuristic(startPoint,dim,matrix)
    path,expandedNodes,trackingpath,cost = BIASTAR.biastar_search(dim , startPoint , goalPoint , matrix, ForwardEuclideanHeuristicMatrix,BackwardEuclideanHeuristicMatrix)

elif algorithm == 'IDASTAR':
    euclideanHeuristicMatrix = calcEuclideanHeuristic(goalPoint,dim,matrix)
    path,expandedNodes,trackingpath,cost = IDASTAR.idastar_search(sys.maxsize, dim, startPoint, goalPoint, matrix, euclideanHeuristicMatrix)


stop = timeit.default_timer()
runTime = stop - start

if runTime < limitedRunTime:
    print(path)
    print('Time:', runTime,'seconds')
else:
    print('FAILED')

output = open("outPut.txt", "w")
trackPath = findTrackingpath(trackingpath)

trackPath = trackPath + ' ' + str(cost) + ' ' + str(expandedNodes)
print(trackPath)
output.write(trackPath)






output.close()











'''
euclideanHeuristicMatrix = calcEuclideanHeuristic(goalPoint,dim,matrix)
#print(euclideanHeuristicMatrix)
path,expandedNodes = ASTAR.astar_search(dim , startPoint , goalPoint , matrix, euclideanHeuristicMatrix)

#stop = timeit.default_timer()

print(path)
#print('Time: ', stop - start)  
print('------------------------------')

#path,expandedNodes = UCS.ucs_search(dim , startPoint , goalPoint , matrix)

#print(path,expandedNodes)

#path = IDS.ids_search(sys.maxsize , dim , startPoint , goalPoint , matrix)

#print(path)

ForwardEuclideanHeuristicMatrix = calcEuclideanHeuristic(goalPoint,dim,matrix)
BackwardEuclideanHeuristicMatrix = calcEuclideanHeuristic(startPoint,dim,matrix)
# =============================================================================
#print(ForwardEuclideanHeuristicMatrix)
#print("-----------------------------")
#print(BackwardEuclideanHeuristicMatrix)
# =============================================================================


path,totalSumG = BIASTAR.biastar_search(dim , startPoint , goalPoint , matrix, ForwardEuclideanHeuristicMatrix,BackwardEuclideanHeuristicMatrix)
print(path)
print(totalSumG)

#path,expandedNodes = UCS.ucs_search(dim , startPoint , goalPoint , matrix)



#path = IDASTAR.idastar_search(sys.maxsize, dim, startPoint, goalPoint, matrix, euclideanHeuristicMatrix)

#print(path)


'''


'''
import sys
output = open(sys.argv[1] + ".txt", "w")
myfile = open(sys.argv[1])
en_data = myfile.read()
print(en_data)
'''           