#!/usr/bin/python

import math as mt

class Position():
	def __init__(self, x = 0.0, y = 0.0, z = 0.0):
		self.x = x
		self.y = y
		self.z = z
	
	def add(self, other):
		return Position(self.x + other.x, self.y + other.y, self.z + other.z)

	def sub(self, other):
		return Position(self.x - other.x, self.y - other.y, self.z + other.y)

	def magSq(self):
		return mt.pow(self.x, 2) + mt.pow(self.y, 2) + mt.pow(self.z, 2)	
	
	def printPos(self):
		print("[{}, {}, {}]".format(self.x, self.y, self.z))	
