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
import timeit

from HeuristicFunctions import calcEuclideanHeuristic
from HeuristicFunctions import calcManhattanHeuristic
from HeuristicFunctions import calcCosineHeuristic



while(1):
    
    print('\nPlease enter the full path of the file input or (exit) to quit\n')
    fileInputName = input()
    
    if(fileInputName == 'exit'):
        break
    
    start = timeit.default_timer()
    
    #Your statements here
    
    def findTrackingpath(trackingpath):
        trackSet = ''
        
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
    
    try:
        f = open(fileInputName, "r")
        
    except IOError:
        print("\מOops!  That was no valid path.  Try again...\n")
        continue
    
    savedInput  = f.read().splitlines()
    algorithm = savedInput[0]
    dim = int(savedInput[1])
    startPoint_list = savedInput[2].split(',')
    startPoint = (int(startPoint_list[0]),int(startPoint_list[0]))
    goalPoint_list = savedInput[3].split(',')
    goalPoint = (int(goalPoint_list[0]),int(goalPoint_list[1]))
    matrix = [[int(num) for num in line.split(',')] for line in savedInput[4:]]
    trackingpath = []
    

    if algorithm == 'ASTAR':

        euclideanHeuristicMatrix = calcEuclideanHeuristic(goalPoint,dim,matrix)
        runningTimeAllowed = (dim/60) * 15
        path,expandedNodes,trackingpath,cost,scannedNodes,PenetrationRatio,minimumDepth,averageDepth,maximumDepth,averageHeuristicValues,ebf,PenetrationY = ASTAR.astar_search(dim , startPoint , goalPoint , matrix, euclideanHeuristicMatrix,runningTimeAllowed,start)

    elif algorithm == 'UCS':
        runningTimeAllowed = (dim/60) * 20
        path,expandedNodes,trackingpath,cost,scannedNodes,PenetrationRatio,minimumDepth,averageDepth,maximumDepth,averageHeuristicValues,ebf,PenetrationY = UCS.ucs_search(dim , startPoint , goalPoint , matrix,runningTimeAllowed,start)
    
    elif algorithm == 'IDS':
        runningTimeAllowed = (dim/60) * 30
        path,expandedNodes,trackingpath,cost,scannedNodes,PenetrationRatio,minimumDepth,averageDepth,maximumDepth,averageHeuristicValues,ebf,PenetrationY = IDS.ids_search(sys.maxsize , dim , startPoint , goalPoint , matrix,runningTimeAllowed,start)
    
    elif algorithm == 'BIASTAR':
        ForwardEuclideanHeuristicMatrix = calcEuclideanHeuristic(goalPoint,dim,matrix)
        BackwardEuclideanHeuristicMatrix = calcEuclideanHeuristic(startPoint,dim,matrix)
        runningTimeAllowed = (dim/60) * 10
        path,expandedNodes,trackingpath,cost,scannedNodes,PenetrationRatio,minimumDepth,averageDepth,maximumDepth,averageHeuristicValues,ebf,PenetrationY = BIASTAR.biastar_search(dim , startPoint , goalPoint , matrix, ForwardEuclideanHeuristicMatrix,BackwardEuclideanHeuristicMatrix,runningTimeAllowed,start)
    
    elif algorithm == 'IDASTAR':
        euclideanHeuristicMatrix = calcEuclideanHeuristic(goalPoint,dim,matrix)
        runningTimeAllowed = (dim/60) * 350
        path,expandedNodes,trackingpath,cost,scannedNodes,PenetrationRatio,minimumDepth,averageDepth,maximumDepth,averageHeuristicValues,ebf,PenetrationY = IDASTAR.idastar_search(sys.maxsize, dim, startPoint, goalPoint, matrix, euclideanHeuristicMatrix,runningTimeAllowed,start)
    
    else :
        
        print('Alogirithm is not defined\n')

    stop = timeit.default_timer()
    runTime = stop - start
    
    pathFile = open(fileInputName + "Path.txt", "w")
    averageStatisticsFile = open("fileInputName + Averagestatistics.txt", "w")

    if runTime > runningTimeAllowed:
        pathFile.write('FAILED\n')
        
    if trackingpath == -1:
         pathFile.write('FAILED\n')
         
    AverageStatisticsValues = '\n' + str(fileInputName) + '             Euclidean Heuristic' + '           ' + str(scannedNodes)+ '             ' + str('%.5f' % PenetrationRatio)+ '                 ' + str('%.5f' % PenetrationY)+ '                  '+str('%.5f' % runTime)+ '                   ' + str('%.5f' % ebf) + '               '+ str('%.5f' % averageHeuristicValues)+ '             ' + str('%.5f' % minimumDepth)+ '          ' + str('%.5f' % averageDepth)+ '              ' + str('%.5f' % maximumDepth)
    #print('Time:', runTime,'seconds\n')      
    
   
    
    if trackingpath != -1:
        trackPath = findTrackingpath(trackingpath)
        trackPath = trackPath + ' ' + str(cost) + ' ' + str(expandedNodes) + '\n'
        #print(trackPath +'\n')
        pathFile.write(trackPath) 
        
    AverageStatistics ='Problem     |      Heuristic Name       |      N      |         d/N           |     Success(Y/N)     |        Time(sec)           |        EBF         |     avg H Value      |      Min     |        Avg            |       Max\n'
    averageStatisticsFile.write(AverageStatistics)
    averageStatisticsFile.write(AverageStatisticsValues)


    
    pathFile.close()
    averageStatisticsFile.close()

    