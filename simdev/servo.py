#!/usr/bin/python

from __future__ import division

import time
import numpy as np
import math as mt
from position import Position

import Adafruit_PCA9685


def map( x, in_min, in_max, out_min, out_max):
	return (x - in_min) * (out_max - out_min)/(in_max - in_min) + out_min


class Servo:	
	
	################################################################
	# Servo Constructor
	################################################################
	def __init__(self, pwm, _id, min_pulse_val, max_pulse_val, base_angle, base_dist, plat_angle, plat_dist, b_deg, inverse):
			
		self.pwm = pwm
		
		self.id = _id
		
		self.min_pulse_val = min_pulse_val
		self.max_pulse_val = max_pulse_val
		
		self.base_angle = base_angle
		self.base_dist = base_dist
		
		self.B = Position()
	
		self.plat_angle = plat_angle
		self.plat_dist = plat_dist
		
		self.P = Position()				
	
		self.inverse = inverse
		
		self.zero = int(map(0, -1000, 1000, self.min_pulse_val, self.max_pulse_val))
		
		self.pos = 0.0

		self.alpha = 0.0	
		self.beta = mt.radians(b_deg)
		
		self.Q = Position()
		self.L = Position()
	
		self._min = -90 if self.inverse else 90
		self._max = 90 if self.inverse else 270 			

	###############################################################
	# Calculates X,Y,Z coordinates of base and platform points
	###############################################################
	def set_coords(self, z_height):
		
		# Base coordinates
		self.B.x = int(self.base_dist * mt.sin(mt.radians(self.base_angle)))	# Bx
		self.B.y = int(self.base_dist * mt.cos(mt.radians(self.base_angle)))	# By			
		self.B.z = 0.0										# Bz

		# Platform coordinates
		self.P.x = int(self.plat_dist * mt.sin(mt.radians(self.plat_angle)))	# Px
		self.P.y = int(self.plat_dist * mt.cos(mt.radians(self.plat_angle)))	# Py
		self.P.z = z_height									# Pz

	###############################################################
	# Sets position of servos directly from controller (testing)
	###############################################################
	def set_pos_direct(self, raw_val):
		
		#mult = -10000 if self.inverse else 10000
		eng_val = map(raw_val , self._min, self._max, self.min_pulse_val, self.max_pulse_val)
		self.pwm.set_pwm(self.id, 0, int(eng_val))
		#print("Servo {} : Pulse {}".format(self.id, eng_val))
		
	###############################################################
	# Sets position of servos from kinematics
	###############################################################
	def set_pos(self):
		self.pwm.set_pwm(self.id, 0, self.pos)
