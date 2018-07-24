#!/usr/bin/python

from controller import Controller

c = Controller()

c.controller_init()
p = 0.0
r = 0.0
y = 0.0

while True:
	p, r, y = c.read_input()
	print 'p = ' + p + ' r = ' + r + ' y = ' + y
	
