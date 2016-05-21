#!/usr/bin/env python
import rospy
import random as r
import math
import numpy as np
from read_config import read_config
from std_msgs.msg import String, Float32, Bool
import heapq
#from copy import deepcopy, copy

class cell(object):
	def _ini_(self, x, y, wall, h, uniform_cost):
		self.x = x
		self.y = y
		self.wall = wall
		self.h = h
		self.uniform_cost = uniform_cost
		self.total_cost = h + uniform_cost

def astar ():
	self.config = read_config()
	self.move_list = self.config['move_list']
	self.start = self.config['start']
	self.goal = self.config['goal']
	self.walls = self.config['walls']
	self.move_list = self.config['pits']
	self.map_size = self.config['map_size']
	self.row_height = self.map_size[0]	
	self.col_width = self.map_size[1]	

	self.heuristics_grid = [self.row][self.col]	
	self.heuristics()
	
	self.uniform_grid = [self.row][self.col]	
	self.uniform()
	
	self.heap = []
	self.visited = []

	self.astar_func()

def heuristics (self, cell):
	for i in range (self.row):
		for j in range (self.col):
			value1 = math.fabs(i - self.goal[0])
			value2 = math.fabs(j - self.goal[1])
			self.heuristics_grid[i][j] = value1 + value2

def uniform(self, cell):
	for i in range (self.row):
		for j in range (self.col):
			value1 = math.fabs(i - self.start[0])
			value2 = math.fabs(j - self.start[1])
			self.uniform_grid[i][j] = value1 + value2

def get_cell(self, x, y):
	for i in range (len(self.heap)):
		if self.heap[i].x == x and self.heap[i].y == y:
			return self.heap[i]

def astar_func():
	#inital all celss
	for i in range (self.row):
		for j in range (self.col):
			#check if is wall
			if (i, j) in walls:
				wall = True
			else
				wall = False
			#create a temp cell
			temp_cell = cell(i, j, wall, self.heuristics_graid[i][j], self.uniform_grid[i][j])
			self.heap.append(temp_cell)

	
	self.visited.append(self.start)
	



