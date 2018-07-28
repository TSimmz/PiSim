#!/usr/bin/python
from __future__ import division


from serial import Serial
from threading import Thread
from controller import Controller
from servo import Servo

import time

import Adafruit_PCA9685


###########################################
# define argument variables
###########################################

###########################################
# initialize global variables
###########################################
p = 0.0
r = 0.0
y = 0.0

# Configure min and max servo pulse lengths
servo_min = 150  # Min pulse length out of 4096
servo_max = 600  # Max pulse length out of 4096

pwm = Adafruit_PCA9685.PCA9685()

# Set frequency to 60hz, good for servos.
pwm.set_pwm_freq(60)

servo_list = []
for i in range(6):
	if i % 2 == 0:
		servo = Servo(pwm, servo_min, servo_max, False)
	else:
		servo = Servo(pwm, servo_min, servo_max, True)

	servo_list.append(servo)

###########################################
# Helper function to map controller values
#  to servo values
###########################################
#def map(x, in_min, in_max, out_min, out_max):
#	return (x - in_min) * (out_max - out_min)/(in_max - in_min) + out_min

###########################################
# input 
#   calculate pitch, roll, yaw from 
#   controller input
###########################################
def input(threadname):
	global p
	global r
	global y
    
	print("Starting input thread...")
	ds4 = Controller()	
	ds4.controller_init()
	while True:
		p, r, y = ds4.read_input()
		

###########################################
# output
#   pitch, roll, yaw to arduino
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
        
    # add eventlisten
    # add functionality to only start threads once motion&control button enabled
    
	input_thread = Thread(target=input, args=("input_thread",))
    #calculate_thread = Thread(target=calculate, args=("calculate_thread",))
    
	input_thread.start()
    #calculate_thread.start()

	# Set frequency to 60hz, good for servos.
	pwm.set_pwm_freq(60)

	while True:
		print("Pitch: {:>6.3f}  Roll: {:>6.3f}  Yaw:{:>6.3f}".format(p, r, y))
		
		#pitch_val = map(p* 1000, -1000, 1000, servo_min, servo_max)
		#roll_val = map(r* 1000, -1000, 1000, servo_min, servo_max)
		#yaw_val = map(y* 1000, -1000, 1000, servo_min, servo_max)
		
		
		#pwm.set_pwm(3, 0, int(pitch_val))
		#pwm.set_pwm(4, 0, int(roll_val))
		#pwm.set_pwm(5, 0, int(yaw_val))
		
		#yaw_val = map(y * 1000, -1000, 1000, servo_min, servo_max)
        #pwm.set_pwm(5, 0, int(yaw_val))
		
		i = 0
		for s in servo_list:
			s.set_position(i,p)
			i = i + 1
    

###########################################
# execute main 
###########################################
if __name__ == "__main__":
	main()
