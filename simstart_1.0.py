#!/usr/bin/python
import pygame
from utils import Utils
from serial import Serial
from threading import Thread

###########################################
# define argument variables
###########################################

###########################################
# initialize global variables
###########################################
p = 0.0
r = 0.0
y = 0.0

###########################################
# input 
#   calculate pitch, roll, yaw from 
#   controller input
###########################################
def input(threadname):
    global p
    global r
    global y
    
    pygame.init()
    clock = pygame.time.Clock()
    pygame.joystick.init()
    
        
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    
    axes = joystick.get_numaxes()
        
    print("Starting input thread...")
    while True:

        pygame.event.pump()

        
        for i in range( axes ):
            axis = joystick.get_axis(i)
            
            if i == 0:
                r = axis
            elif i == 1:
                p = axis
            elif i == 3:
                y = axis
                    
        #print("Pitch: {:>6.3f}  Roll: {:>6.3f}  Yaw:{:>6.3f}".format(p, r, y))
        clock.tick(20)


###########################################
# output
#   pitch, roll, yaw to arduino
###########################################
def output(threadname):
    
    print("Starting output thread...")
    
    while True:
        print("Pitch: {:>6.3f}  Roll: {:>6.3f}  Yaw:{:>6.3f}".format(p, r, y))
          

    #set up output serial port to output positional information to arduino

###########################################
# main 
#   initialize child threads
###########################################
def main():
    
    print("Starting main thread...")
    
    
    # add eventlisten
    # add functionality to only start threads once motion&control button enabled
    
    input_thread = Thread(target=input, args=("input_thread",))
    #output_thread = Thread(target=output, args=("output_thread",))
    
    input_thread.start()
    #output_thread.start()
    
##    i = 0
##    while True:
##        i = i+1
##        
##        if i==1000:
##            i = 0

    while True:
        #print "Hello"
	print("Pitch: {:>6.3f}  Roll: {:>6.3f}  Yaw:{:>6.3f}".format(p, r, y))


    

###########################################
# execute main 
###########################################
if __name__ == "__main__":
    main()
