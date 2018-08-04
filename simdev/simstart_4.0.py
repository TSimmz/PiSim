#!/usr/bin/python
from __future__ import division

import sys
from serial import Serial
from threading import Thread
from controller import Controller
from servo import Servo

import numpy as np
import math as mt

import time

import Adafruit_PCA9685

###########################################
# initialize global variables
###########################################
p = 0.0
r = 0.0
y = 0.0

###########################################
# Constants (measurements in mm)
###########################################
SERVO_MIN = 150 	# Min pulse length out of 4096
SERVO_MAX = 600  	# Max pulse length out of 4096
BASE_DIST = 122.1	# From center to servo pivot center
PLAT_DIST = 140.5	# From center to joint pivot center
SERVO_LEN = 40.0 	# Length of servo arm
SERVO_DIST = 162.8 	# From center to servo arm pivot center
LEG_LEN = 182.0 	# Length of leg from base to platform
Z_HOME = 191.5		# Height of platform above base
SERVO_LIST = []		# List of servo class objects

###########################################
# Create PWM object to drive servos
###########################################
PWM = Adafruit_PCA9685.PCA9685()

###########################################
# Create Controller object
###########################################
DS4 = Controller()

###########################################
# Platform code
###########################################
PI = mt.pi
DEG2RAD = 180 / mt.pi
DEG30 = mt.pi / 6

platform_pos = np.array([0.0, 0.0, 0.0, mt.radians(0), mt.radians(0), mt.radians(0)]) # Requested position of platform

ROT = np.zeros((3,3))			# Rotational matrix
T = np.zeros(3)					# Translational matrix
H = np.array([0, 0, Z_HOME])	# Center position of platform

###########################################
# Helper function to calculate needed servo
#  rotation value
###########################################
def getAlpha(i):
	
	n = 0
	th = 0.0
	q = np.zeros(3)
	d1 = np.zeros(3)
	d12 = 0
	_min = ser_min
	_max = ser_max
	th = theta_a[i]
	
	while n < 20:

		# Calculation of position of base attachment point
		# (Point on servo arm where leg is connected
		q[0] = L1*mt.cos(th)*mt.cos(beta[i]) + P[0][i]
		q[1] = L1*mt.cos(th)*mt.sin(beta[i]) + P[1][i]
		q[2] = L1*mt.sin(th)

		# Calculation of distance between according platform attachment
		# point and base attachment point
		d1[0] = RXP[0][i] - q[0]
		d1[1] = RXP[1][i] - q[1]
		d1[2] = RXP[2][i] - q[2]

		dl2 = mt.sqrt(d1[0]*d1[0] + d1[1]*d1[1] + d1[2]*d1[2])
		
		# If distance is same as leg length, value of theta_a is correct
		if mt.fabs(L2-d12) < 0.01:
			return th
		
		# If not, split the searched space in half and try again
		if d12 < L2:
			_max = th
		else:
			_min = th

		n += 1
		
		if _max == ser_min or _min == ser_max:
			return th

		th = _min + (_max-_min) / 2
	
	return th

###########################################
# Function to calculate rotational matrix
###########################################
def getRotMatrix(plt):
	
	psi = plt[5]		# yaw
	theta = plt[4]		# pitch
	phi = plt[3]		# roll
	
	# Rotational matrix value calculation	
	ROT[0][0] = mt.cos(psi)*mt.cos(theta)
	ROT[1][0] = -mt.sin(psi)*mt.cos(phi)+mt.cos(psi)*mt.sin(theta)*mt.sin(phi)
	ROT[2][0] = mt.sin(psi)*mt.sin(phi)+mt.cos(psi)*mt.cos(phi)*mt.sin(theta)

	ROT[0][1] = mt.sin(psi)*mt.cos(theta)
	ROT[1][1] = mt.cos(psi)*mt.cos(phi)+mt.sin(psi)*mt.sin(theta)*mt.sin(phi)
	ROT[2][1] = -mt.cos(psi)*mt.sin(phi)+mt.sin(psi)*mt.sin(theta)*mt.cos(phi)

	ROT[0][2] = -mt.sin(theta)
	ROT[1][2] = -mt.cos(theta)*mt.sin(psi)
	ROT[2][2] = mt.cos(theta)*mt.cos(phi)

	print ROT
###########################################
# Function calculate wanted position of
# platform attachment points using rot
# matrix and tranlation vector
###########################################
def getRxp():
		
	for i in range(6):
		RXP[0][i] = T[0]+ROT[0][0]*(RE[0][i])+M[0][1]*(RE[1][i])+M[0][2]*(RE[2][i])
		RXP[1][i] = T[1]+ROT[1][0]*(RE[0][i])+M[1][1]*(RE[1][i])+M[1][2]*(RE[2][i])
		RXP[2][i] = T[2]+ROT[2][0]*(RE[0][i])+M[2][1]*(RE[1][i])+M[2][2]*(RE[2][i])

###########################################
# Function calculating Q for all legs
###########################################
def getQ():
	
	for s in SERVO_LIST:
		print ROT
		#print
		#P = np.matmul(ROT, 
		

###########################################
# Function calculating translation vector
###########################################
def getT(pe):
	T[0] = pe[0]+H[0]
	T[1] = pe[1]+H[1]
	T[2] = pe[2]+H[2]

###########################################
# Function to set the postion of servos
###########################################
def setPos(pe):
	errCount = 0
	#print pe
	for i in range(6):
		getT(pe)
		getMatrix(pe)
		getRxp(pe)
		
		theta_a[i] = getAlpha(i)
		print theta_a[i]	
		if i % 2 == 0:
			servo_pos[i] = constrain(zero[i] - (theta_a[i])*servo_mult, SERVO_MIN, SERVO_MAX)
		else:
			servo_pos[i] = constrain(zero[i] + (theta_a[i])*servo_mult, SERVO_MIN, SERVO_MAX)				
		
	#print servo_pos

	for i in range(6):
		if theta_a[i] == ser_min or theta_a[i] == ser_max or servo_pos[i] == SERVO_MIN or servo_pos[i]== SERVO_MAX:
			errCount += 1


		servo_list[i].set_pos(i, servo_pos[i])

	
	#print servo_pos
	return errCount
	
###########################################
# Helper function to map controller values
#  to servo values
###########################################
def map(x, in_min, in_max, out_min, out_max):
	return (x - in_min) * (out_max - out_min)/(in_max - in_min) + out_min

###########################################
# Helper function to limit values to btwn
#  a range
###########################################
def constrain(val, min_val, max_val):
    return min(max_val, max(min_val, val))


###########################################
# Helper function to initialize all servos
###########################################
def Initialize_Servos():
	
	PWM.set_pwm_freq(60)
	
	s0 = Servo(PWM, 0, SERVO_MIN, SERVO_MAX,
		308.0, BASE_DIST,
		273.1, PLAT_DIST, False)

	s1 = Servo(PWM, 1, SERVO_MIN, SERVO_MAX,
		352.0, BASE_DIST,
		 26.9, PLAT_DIST, True)

	s2 = Servo(PWM, 2, SERVO_MIN, SERVO_MAX, 
		 68.0, BASE_DIST,
		 33.1, PLAT_DIST, False)
	
	s3 = Servo(PWM, 3, SERVO_MIN, SERVO_MAX,
		112.0, BASE_DIST,
		146.9, PLAT_DIST, True)

	s4 = Servo(PWM, 4, SERVO_MIN, SERVO_MAX,
		188.0, BASE_DIST,
		153.1, PLAT_DIST, False)

	s5 = Servo(PWM, 5, SERVO_MIN, SERVO_MAX,
		232.0, BASE_DIST,
		266.9, PLAT_DIST, True)
 
	SERVO_LIST.append(s0)
	SERVO_LIST.append(s1)
	SERVO_LIST.append(s2)
	SERVO_LIST.append(s3)
	SERVO_LIST.append(s4)
	SERVO_LIST.append(s5)
 
	print("\n###########################################")
	for s in SERVO_LIST: 
		print("Initializing Servo {}".format(s.id))
		s.set_coords(Z_HOME)

		#print("Base coords:")
		#print s.base_coords

		#print("Plat coords:")
		#print s.plat_coords

		time.sleep(0.25)

	print("Servos have been initialized...")
	print("###########################################\n")	

###########################################
# Helper function to initialize controls
###########################################
def Initialize_Controls():
	print("\n############################################")
	if DS4.controller_init():
		print("Controller found...")
		time.sleep(0.25)
	else:
		print("Controller not found...")
		print("############################################\n")
		time.sleep(1)
		sys.exit("Exiting...")

###########################################
# Helper function to initialize variables
###########################################
def Initialize_Vars():
	pass

###########################################
# controls 
#   Retrieve pitch, roll, yaw values from 
#   controller input
###########################################
def controls(threadname):
	global p
	global r
	global y
   
	
	print("Starting controls thread...")
	
	while True:
		DS4.read_input()	
					
		p = DS4.control_map['y']
		r = DS4.control_map['x']
		y = DS4.control_map['rx']

###########################################
# calculate
#  
###########################################
def calculate(threadname):
    
	print("Starting calculation thread...")
    
	while True:
		#setPos(arr)
		pass
	
###########################################
# main 
#   initialize child threads
###########################################
def main():
    
	print("Starting main thread...\n...")

	print("Starting setup...\n...") 
	Initialize_Servos()
  	Initialize_Controls()
	Initialize_Vars()
	
	controls_thread = Thread(target=controls, args=("controls_thread",))
	#calculate_thread = Thread(target=calculate, args=("calculate_thread",))
    
	controls_thread.start()
	#calculate_thread.start()

	PWM.set_pwm_freq(60)	# Set frequency to 60hz
	motion = False			# Used to track motion status

	print("Setup complete!")
	print("Press Start to enable Controls & Motion...")
	time.sleep(0.25)
	
	#SERVO_LIST[0].set_pos(0, -1)
	#SERVO_LIST[1].set_pos(1, -1)
	#SERVO_LIST[2].set_pos(2, 1)
	#SERVO_LIST[3].set_pos(3, 1)
	#SERVO_LIST[4].set_pos(4, -1)
	#SERVO_LIST[5].set_pos(5, -1)
	
	while True:
		#print("Pitch: {:>6.3f}  Roll: {:>6.3f}  Yaw:{:>6.3f}\r".format(p, r, y)),
		
		#if motion:
		i = 0
		for s in SERVO_LIST:
			s.set_pos_direct(i,p)
			i = i + 1
		
		#if DS4.control_map['start']:
		#	time.sleep(0.1333)
		#	motion = not motion
		#	if motion:
		#		print("Starting motion...")
		#	else:
		#		print("Stoping motion...")

###########################################
# Execute main 
###########################################
if __name__ == "__main__":
	main()
