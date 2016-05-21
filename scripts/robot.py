#!/usr/bin/env python

import rospy
import random
import math
import numpy as np
from std_msgs.msg import String, Float32, Bool
from read_config import read_config
from astar import *
from mdp import *

class Robot ():
	def __init__(self):
		rospy.init_node ("robot")
		self.config = read_config()
		self.astar_pub = rospy.Publisher(
			"/results/path_list",
			AStarPath,
			queue_size = 10
		)

		self.sim_complete_pub = rospy.Publisher(
			"/map_node/sim_complete",
			Bool,
			queue_size = 10
		)

		# publish A* path
		self.publish_astar()
		mdp()

		rospy.sleep(1)
		self.sim_complete_pub.publish(True)
		rospy.sleep(1)
		rospy.signal_shutdown(Robot)


	def publish_astar(self):
		# astar should return []
		path_array = astar()
		for i in range (len(path_array)):
			self.astar.publish(path_array[i])


if __name__ == '__main__':
   r = Robot()