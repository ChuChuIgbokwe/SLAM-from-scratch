#!/usr/bin/env python
# -*- coding: utf-8 -*-


from pylab import *
from lego_robot import LegoLogfile
'''
Plot the increments of the left and right motor.
01_c_plot_motor_increments.py
Claus Brenner, 07 NOV 2012
'''

if __name__ == '__main__':

    logfile = LegoLogfile()
    logfile.read("robot4_motors.txt")

    plot(logfile.motor_ticks)
    show()
