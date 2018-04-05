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
    
    print("Starting input thread...")
    while True:

        pygame.event.pump()
        joystick_count = pygame.joystick.get_count()
        
        for i in range(joystick_count):
            joystick = pygame.joystick.Joystick(i)
            joystick.init()
            
            axes = joystick.get_numaxes()
            for i in range( axes ):
                axis = joystick.get_axis(i)
                if i == 0:
                    r = float(axis)
                elif i == 1:
                    p = float(axis)
                elif i == 3:
                    y == float(axis)
        #clock.tick(60)


###########################################
# output
#   pitch, roll, yaw to arduino
###########################################
def output(threadname):
    
    print("Starting output thread...")
    
    while True:
        print("Pitch: {}  Roll: {}  Yaw:{}".format(p, r, y))
          

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
    output_thread = Thread(target=output, args=("output_thread",))
    
    input_thread.start()
    output_thread.start()
    i = 0
    while True:
        i = i+1
        
        if i==1000:
            i = 0



    

###########################################
# execute main 
###########################################
if __name__ == "__main__":
    main()