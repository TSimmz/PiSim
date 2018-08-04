#!/usr/bin/python

from __future__ import division

import time
import numpy as np
import math as mt
import Adafruit_PCA9685


def map( x, in_min, in_max, out_min, out_max):
	return (x - in_min) * (out_max - out_min)/(in_max - in_min) + out_min


class Servo:	
	
	################################################################
	# Servo Constructor
	################################################################
	def __init__(self, pwm, _id, min_pulse_val, max_pulse_val, base_angle, base_dist, plat_angle, plat_dist, inverse):
			
		self.pwm = pwm
		
		self.id = _id
		
		self.min_pulse_val = min_pulse_val
		self.max_pulse_val = max_pulse_val
		
		self.base_angle = base_angle
		self.base_dist = base_dist
		self.base_coords = np.zeros((3,1))
	
		self.plat_angle = plat_angle
		self.plat_dist = plat_dist
		self.plat_coords = np.zeros((3,1))				
	
		self.inverse = inverse
		
		self.zero = int(map(0, -1000, 1000, self.min_pulse_val, self.max_pulse_val))
		
		self.pos = 0.0

		self.alpha = 0.0	
		self.beta = mt.radians(self.base_angle)
		
		self.Q = np.zeros((3,1))
		self.L = np.zeros((3,1))
				

	###############################################################
	# Calculates X,Y,Z coordinates of base and platform points
	###############################################################
	def set_coords(self, z_height):
		
		# Base coordinates
		self.base_coords[0,0] = int(self.base_dist * mt.sin(self.beta))	# Bx
		self.base_coords[1,0] = int(self.base_dist * mt.cos(self.beta))	# By		
		self.base_coords[2,0] = 0										# Bz

		# Platform coordinates
		self.plat_coords[0,0] = int(self.plat_dist * mt.sin(self.beta))	# Px
		self.plat_coords[1,0] = int(self.plat_dist * mt.cos(self.beta))	# Py
		self.plat_coords[2,0] = z_height								# Pz

	###############################################################
	# Sets position of servos directly from controller (testing)
	###############################################################
	def set_pos_direct(self, raw_val):
		
		mult = -1000 if self.inverse else 1000
		eng_val = map(raw_val * mult, -1000, 1000, self.min_pulse_val, self.max_pulse_val)
		self.pwm.set_pwm(self.id, 0, int(eng_val))
		
	###############################################################
	# Sets position of servos from kinematics
	###############################################################
	def set_pos(self, raw_val):
		self.pwm.set_pwm(self.id, 0, int(raw_val))


