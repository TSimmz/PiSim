#!/usr/bin/python
from threading import Thread

###########################################
# define argument variables
###########################################

###########################################
# initialize global variables
###########################################
p = 0
r = 0
y = 0

###########################################
# input 
#   calculate pitch, roll, yaw from 
#   controller input
###########################################
def input(threadname):
    global p
    global r
    global y

    pass

###########################################
# output
#   pitch, roll, yaw to arduino
###########################################
def output(threadname):
    pass

###########################################
# main 
#   initialize child threads
###########################################
def main():
    input_thread = Thread(target=input, args=("input_thread",))
    output_thread = Thread(target=output, args=("output_thread"))



###########################################
# execute main 
###########################################
if __name__ == "__main__":
    main():