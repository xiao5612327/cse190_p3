#!/usr/bin/env python
import rospy
import random as r
import math
import numpy as np
from read_config import read_config
from std_msgs.msg import String, Float32, Bool
import heapq
#from copy import deepcopy, copy


def astar ():
	self.config = read_config()
	self.move_list = self.config['move_list']
	self.start = self.config['start']
	self.goal = self.config['goal']
	self.walls = self.config['walls']
	self.move_list = self.config['pits']
	self.map_size = self.config['map_size']
	self.row = self.map_size[0]	
	self.col = self.map_size[1]	

	self.heuristics_grid = [self.row][self.col]	
	self.heuristics()

	self.heap = []
	self.astar_func()

def heuristics ():
	for i in range (self.row):
		for j in range (self.col):
			value1 = math.fabs(i - self.goal[0])
			value2 = math.fabs(j - self.goal[1])
			self.heuristics_grid[i][j] = value1 + value2

	
def astar_func():
	for i in range (self.row):
		for j in range (self.col):
			self.heap.append( (,i,j) )

	



