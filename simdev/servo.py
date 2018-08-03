#!/usr/bin/python

from __future__ import division

import time
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
		
		self.plat_angle = plat_angle
		self.plat_dist = plat_dist
				
		self.inverse = inverse
	
	###############################################################
	# Calculates X,Y,Z coordinates of base and platform points
	###############################################################
	def set_coords(self, z_height):
		self.base_coords = {
			'x': int(self.base_dist * mt.sin(mt.radians(self.base_angle))),
			'y': int(self.base_dist * mt.cos(mt.radians(self.base_angle))),
			'z': 0}
		self.plat_coords = {
			'x': int(self.plat_dist * mt.sin(mt.radians(self.plat_angle))),
			'y': int(self.plat_dist * mt.cos(mt.radians(self.plat_angle))),
			'z': z_height}	
	
	###############################################################
	# Sets position of servos directly from controller (testing)
	###############################################################
	def set_pos_direct(self, channel, raw_val):
		
		mult = -1000 if self.inverse else 1000
		eng_val = map(raw_val * mult, -1000, 1000, self.min_pulse_val, self.max_pulse_val)
		self.pwm.set_pwm(channel, 0, int(eng_val))
		
	###############################################################
	# Sets position of servos from kinematics
	###############################################################
	def set_pos(self, channel, raw_val):
		
		self.pwm.set_pwm(channel, 0, int(raw_val))
