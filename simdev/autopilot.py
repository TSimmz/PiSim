#!/usr/bin/python

import time
from position import Position

class Autopilot():
	def __init__(self, dist, home):
		self.new = Position()
		
		self.dist = dist
		self.home = home
	
	#########################################
	# Resets to start position
	#########################################
	def reset(self):
		self.new = Position()

	#########################################
	# Rotate pitch between min and max
	#########################################
	def pitch(self):
		pass

	#########################################
	# Rotate roll between min and max
	#########################################
	def roll(self):
		pass

	#########################################
	# Rotate yaw between min and max
	#########################################
	def yaw(self):
		pass

	#########################################
	# Translate z-axis between min and max
	#########################################
	def bounce(self):
		pass
	#########################################
	# Translate x-axis between min and max
	#########################################
	def lateral(self):
		pass
	
	#########################################
	# Translate y-axis between min and max
	#########################################
	def longitude(self):
		pass
	
	#########################################
	# Translate in a circle at Z_home
	#########################################
	def circle_path(self):
		pass
	
		
