#!/usr/bin/env python
# -*- coding: utf-8 -*-

from lego_robot import LegoLogfile
'''
Print the increments of the left and right motor.
Now using the LegoLogfile class.
01_b_print_motor_increments.py
Claus Brenner, 07 NOV 2012
'''

if __name__ == '__main__':

    logfile = LegoLogfile()
    logfile.read("robot4_motors.txt")

    for i in range(20):
        print logfile.motor_ticks[i]
