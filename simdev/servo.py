#!/usr/bin/python

from __future__ import division

import time
import Adafruit_PCA9685


def map( x, in_min, in_max, out_min, out_max):
	return (x - in_min) * (out_max - out_min)/(in_max - in_min) + out_min


class Servo:	
	def __init__(self, pwm, id, min_pulse_val, max_pulse_val, base_angle, base_dist,  plat_angle, plat_dist, inverse):
			
		self.pwm = pwm
		
		self.id = id
		
		self.min_pulse_val = min_pulse_val
		self.max_pulse_val = max_pulse_val
		
		self.base_angle = base_angle
		self.base_dist = base_dist

		self.plat_angle = plat_angle
		self.plat_dist = plat_dist
		
		self.inverse = inverse
			
	def set_pos_direct(self, channel, raw_val):
		mult = -1000 if self.inverse else 1000
		eng_val = map(raw_val * mult, -1000, 1000, self.min_val, self.max_val)
		self.pwm.set_pwm(channel, 0, int(eng_val))

	def set_pos(self, channel, raw_val):
		self.pwm.set_pwm(channel, 0, int(raw_val))
