#!/usr/bin/env python

import rospy
import random
import math
import numpy as np
from std_msgs.msg import String, Float32, Bool
from read_config import read_config
from cse_190_assi_3.msg import *
from astar import *

class MDP():
	def __init__ (self):
		self.config = read_config()
		self.move_list = self.config['move_list']

		self.max_iterations = self.config['max_iterations']
		self.threshold_difference = self.config['threshold_difference']
		self.step_reward = self.config['reward_for_each_step']
		self.wall_reward = self.config['reward_for_hitting_wall']
		self.goal_reward = self.config['reward_for_reaching_goal']
		self.pit_reward = self.config['reward_for_falling_in_pit']

		
		self.discount_factor = self.config['discount_factor']
		self.prob_move_forward = self.config['prob_move_forward']
		self.prob_move_backward = self.config['prob_move_backward']
		self.prob_move_left = self.config['prob_move_left']
		self.prob_move_right = self.config['prob_move_right']

		self.start = self.config['start']
		self.goal = self.config['goal']
		self.walls = self.config['walls']
		self.pits = self.config['pits']
		self.map_size = self.config['map_size']
		self.row = self.map_size[0]	
		self.col = self.map_size[1]	

		self.mdp_pub = rospy.Publisher(
			"/results/policy_list",
			PolicyList,
			queue_size = 10
		)



	def make_policy (self):
		return	
