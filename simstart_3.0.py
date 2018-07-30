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
# Set min and max servo pulse lengths
###########################################
SERVO_MIN = 150  # Min pulse length out of 4096
SERVO_MAX = 600  # Max pulse length out of 4096

###########################################
# Create PWM object to drive servos
###########################################
PWM = Adafruit_PCA9685.PCA9685()

###########################################
# Create Controller object
###########################################
DS4 = Controller()

###########################################
# Set frequency to 60hz, good for servos.
###########################################
PWM.set_pwm_freq(60)

###########################################
# List of servos
###########################################
servo_list = []

###########################################
# Platform code
###########################################

DEG2RAD = 180 / mt.pi
DEG30 = mt.pi / 6

# Zero pos of servos
zero = np.array([209, 209, 209, 209, 209, 209])
# Requested position for platform
arr = np.array([0.0, 0.0, 0.0, mt.radians(0), mt.radians(0), mt.radians(0)])
# Actual degree of rotation of servo arms
theta_a = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
# Current servo positions
servo_pos = np.empty([6])
# Rotation of servo arms in respect to x-axis
beta = np.array([mt.pi/2, -mt.pi/2, -mt.pi/6, 5*mt.pi/6, -5*mt.pi/6, mt.pi/6])
# Min/Max servo positions
ser_min = mt.radians(-80)
ser_max = mt.radians(80)
# mulitpler used for conversion radians -> servo pulse
servo_mult = 400/(mt.pi/4)

###########################################
###########################################
#      SIM SPECIFIC - MUST ADJUST         #
###########################################
###########################################

# Effective lenth of servo arm
L1 = 0.79
# Length of base and platform connecting arm
L2 = 4.66
# Height of platform above base
Z_HOME = 4.05
# Distance from center of platform to attachment points
RD = 2.42
# Distance from center of base to center of servo rotation points
PD = 2.99
# Angle between two servo axis points
THETA_P = mt.radians(37.5)
# Angle between platform attachment points
THETA_R = mt.radians(8)
# Helper variable
THETA_ANGLE = ((mt.pi/3)-THETA_P)/2.0
# X-Y Values for Servo rotation points
P = np.array((
	[int(-PD * mt.cos(DEG30 - THETA_ANGLE)),
	int(-PD * mt.cos(DEG30 - THETA_ANGLE)),
	int(PD * mt.sin(THETA_ANGLE)),
	int(PD * mt.cos(DEG30 + THETA_ANGLE)),
	int(PD * mt.cos(DEG30 + THETA_ANGLE)),
	int(PD * mt.sin(THETA_ANGLE))],
	[int(-PD * mt.sin(DEG30 - THETA_ANGLE)),
	int(PD * mt.sin(DEG30 - THETA_ANGLE)),
	int(PD * mt.cos(THETA_ANGLE)),
	int(PD * mt.sin(DEG30 + THETA_ANGLE)),
	int(-PD * mt.sin(DEG30 - THETA_ANGLE)),
	int(-PD * mt.cos(THETA_ANGLE))]))
# X-Y-Z Values of platform attachment points positions
R = np.array((
	[int(-RD * mt.sin(DEG30 + THETA_R/2)),
	int(-RD * mt.sin(DEG30 + THETA_R/2)),
	int(-RD * mt.sin(DEG30 - THETA_R/2)),
	int(RD * mt.cos(THETA_R/2)),
	int(RD * mt.cos(THETA_R/2)),
	int(-RD * mt.sin(DEG30 - THETA_R/2))],
	[int(-RD * mt.cos(DEG30 + THETA_R/2)),
	int(RD * mt.cos(DEG30 + THETA_R/2)),
	int(RD * mt.cos(DEG30 + THETA_R/2)),
	int(RD * mt.sin(THETA_R/2)),
	int(-RD * mt.sin(THETA_R/2)),
	int(-RD * mt.cos(DEG30 - THETA_R/2))],
	[0,0,0,0,0,0]))
	
# Arrays used for servo rotation calculation
M = np.zeros((3,3))
RXP = np.zeros((3,6))
T = np.zeros(3)
# Center position of platform
H = np.array([0, 0, Z_HOME])

###########################################
# Helper function to calculate needed servo
#  rotation value
###########################################
def GetAlpha(i):
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
		q[0] = L1*mt.cos(th)*cos(beta[i]) + P[0][i]
		q[1] = L1*mt.cos(th)*mt.sin(beta[i]) + P[1][i]
		q[2] = L1*mt.sin(th)

		# Calculation of distance between according platform attachment
		# point and base attachment point
		dl[0] = rxp[0][i] - q[0]
		dl[1] = rxp[1][i] - q[1]
		dl[2] = rxp[2][i] - q[2]
		dl2 = mt.sqrt(dl[0]*dl[0] + dl[1]*dl[1] + dl[2]*dl[2])
		
		# If distance is same as leg length, value of theta_a is correct
		if mt.fabs(L2-d12) < 0.01:
			return th
		
		# If not, split the searched space in half and try again
		if d12 < L2:
			max = th
		else:
			min = th

		n += 1
		
		if max == ser_min or min == ser_max:
			return th

		th = min + (max-min) / 2
	
	return th

###########################################
# Function to calculate rotation matrix
###########################################
def getMatrix(pe):
	psi = pe[5]
	theta = pe[4]
	phi = pe[3]
	
	M[0][0] = mt.cos(psi)*mt.cos(theta)
	M[1][0] = -mt.sin(psi)*mt.cos(phi)+mt.cos(psi)*mt.sin(theta)*mt.sin(phi)
	M[2][0] = mt.sin(psi)*mt.sin(phi)+mt.cos(psi)*mt.cos(phi)*mt.sin(theta)

	M[0][1] = mt.sin(psi)*mt.cos(theta)
	M[1][1] = mt.cos(psi)*mt.cos(phi)+mt.sin(psi)*mt.sin(theta)*mt.sin(phi)
	M[2][1] = mt.cos(theta)*mt.sin(phi)

	M[0][2] = -sin(theta);
	M[1][2] = -cos(psi)*sin(phi)+sin(psi)*sin(theta)*cos(phi);
	M[2][2] = cos(theta)*cos(phi);	

###########################################
# Function calculate wanted position of
# platform attachment points using rot
# matrix and tranlation vector
###########################################
def getRxp(pe):
	
	for i in range(6):
		rxp[0][i] = T[0]+M[0][0]*(re[0][i])+M[0][1]*(re[1][i])+M[0][2]*(re[2][i])
		rxp[1][i] = T[1]+M[1][0]*(re[0][i])+M[1][1]*(re[1][i])+M[1][2]*(re[2][i])
		rxp[2][i] = T[2]+M[2][0]*(re[0][i])+M[2][1]*(re[1][i])+M[2][2]*(re[2][i])

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
	
	for i in range(6):
		getT(pe)
		getMatrix(pe)
		getRxp(pe)
			
		if i % 2 == 0:
			servo_pos[i] = constrain(zero[i] - (theta_a[i])*servo_mult, SERVO_MIN, SERVO_MAX)
		else:
			servo_pos[i] = constrain(zero[i] + (theta_a[i])*servo_mult, SERVO_MIN, SERVO_MAX)				

	for i in range(6):
		if theta_a[i] == ser_min or theta_a[i] == ser_max or servo_pos[i] == SERVO_MIN or servo_pos[i]== SERVO_MAX:
			errCount += 1

		servo_list[i].set_position(i, servo_pos[i])

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
def init_servos():
  for i in range(6):
  	if i % 2 == 0:
  		servo = Servo(PWM, SERVO_MIN, SERVO_MAX, False)
  	else:
  		servo = Servo(PWM, SERVO_MIN, SERVO_MAX, True)
     
	print("Initializing Servo {}".format(i+1))
	time.sleep(1)
	servo_list.append(servo)

  print("Servos have been initialized...")
  
###########################################
# input 
#   Retrieve pitch, roll, yaw values from 
#   controller input
###########################################
def input(threadname):
	global p
	global r
	global y
   
	
	print("Starting input thread...")
	#ds4 = Controller()	
	
	#if (ds4.controller_init()):
	while True:
		p, r, y = DS4.read_input()	
	#else:
	#	sys.exit("Controller not found!")
		

###########################################
# calculate
#  
###########################################
def calculate(threadname):
    
	print("Starting calculation thread...")
    
	#while True:
	#	print("Pitch: {:>6.3f}  Roll: {:>6.3f}  Yaw:{:>6.3f}".format(p, r, y))
     
	


###########################################
# main 
#   initialize child threads
###########################################
def main():
    
	print("Starting main thread...")
 
	init_servos()
  
	if (DS4.controller_init()):
		print("Controller found!")
		time.sleep(1)
	else:
		sys.exit("Controller not found!")
  

    # add eventlisten
    # add functionality to only start threads once motion&control button enabled
    
	input_thread = Thread(target=input, args=("input_thread",))
    #calculate_thread = Thread(target=calculate, args=("calculate_thread",))
    
	input_thread.start()
    #calculate_thread.start()

	# Set frequency to 60hz, good for servos.
	PWM.set_pwm_freq(60)

	
	while True:
		#print("Pitch: {:>6.3f}  Roll: {:>6.3f}  Yaw:{:>6.3f}".format(p, r, y))
				
		i = 0
		for s in servo_list:
			s.set_position(i,p)
			i = i + 1
    

###########################################
# execute main 
###########################################
if __name__ == "__main__":
	main()
