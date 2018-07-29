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
pwm = Adafruit_PCA9685.PCA9685()

###########################################
# Create Controller object
###########################################
ds4 = Controller()

###########################################
# Set frequency to 60hz, good for servos.
###########################################
pwm.set_pwm_freq(60)


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
# Angle between tow servo axis points
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
	



###########################################
# Helper function to map controller values
#  to servo values
###########################################
def map(x, in_min, in_max, out_min, out_max):
	return (x - in_min) * (out_max - out_min)/(in_max - in_min) + out_min


###########################################
# Helper function to initialize all servos
###########################################
def init_servos():
  for i in range(6):
  	if i % 2 == 0:
  		servo = Servo(pwm, SERVO_MIN, SERVO_MAX, False)
  	else:
  		servo = Servo(pwm, SERVO_MIN, SERVO_MAX, True)
     
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
		p, r, y = ds4.read_input()	
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
  
	if (ds4.controller_init()):
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
	pwm.set_pwm_freq(60)

	
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
