# -*- coding: utf-8 -*-
"""
Created on Wed Jan  6 17:16:52 2021

@author: Shadi and Noor
"""

class Node:
      
    
    # Building the Constructor of A Node
    def __init__(self,point:tuple,parent:tuple):
        self.point = point
        self.parent = parent
        self.g = 0 # g cost
        self.h = 0 # h cost
        self.f = 0 # f cost
        self.d = 0


    # Compare nodes
    def __eq__(self, other):
        return self.point == other.point
        
    # Sort nodes
    def __lt__(self, other):        
        if self.f != other.f:
            return self.f < other.f
        elif self.h != other.h:    
            return self.h < other.h
     
    # Print node
    def __repr__(self):
        return ('({0},{1})'.format(self.point, self.f))
    