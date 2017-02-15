#!/usr/bin/env python
# -*- coding: utf-8 -*-


from pylab import *
from lego_robot import *

'''
Compute the derivative of a scan.
03_b_scan_derivative
Claus Brenner, 09 NOV 2012
'''

def compute_derivative(scan, min_dist):
    '''
    Find the derivative in scan data, ignoring invalid measurements.
    :param scan:
    :param min_dist:
    :return: jumps
    '''
    jumps = [ 0 ]
    for i in xrange(1, len(scan) - 1):
        # --->>> Insert your code here.
        # Compute derivative using formula "(f(i+1) - f(i-1)) / 2".
        # Do not use erroneous scan values, which are below min_dist.
        # jumps.append(i%20 * 10) # Replace this line, append derivative instead.
        if scan[i+1]> min_dist and scan[i-1]> min_dist:
            derivative = (scan[i+1]-scan[i-1])/2.0
            jumps.append(derivative)
        else:
            jumps.append(0)

    jumps.append(0)
    return jumps


if __name__ == '__main__':

    minimum_valid_distance = 20.0

    # Read the logfile which contains all scans.
    logfile = LegoLogfile()
    logfile.read("robot4_scan.txt")

    # Pick one scan.
    scan_no = 235
    scan = logfile.scan_data[scan_no]

    # Compute derivative, (-1, 0, 1) mask.
    der = compute_derivative(scan, minimum_valid_distance)

    # Plot scan and derivative.
    title("Plot of scan %d" % scan_no)
    plot(scan)
    plot(der)
    show()
