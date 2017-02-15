#!/usr/bin/env python
# -*- coding: utf-8 -*-

from pylab import *
from lego_robot import *
#got it working 31/03/2016 by 12.05am

'''
For each cylinder in the scan, find its ray and depth.
03_c_find_cylinders
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
        l = scan[i-1]
        r = scan[i+1]
        if l > min_dist and r > min_dist:
            derivative = (r - l) / 2.0
            jumps.append(derivative)
        else:
            jumps.append(0)
    jumps.append(0)
    return jumps


def find_cylinders(scan, scan_derivative, jump, min_dist):
    cylinder_list = []
    on_cylinder = False
    sum_ray, sum_depth, rays = 0.0, 0.0, 0
    for i in xrange(len(scan_derivative)):
        if scan_derivative[i] < -jump:
            on_cylinder = True
            print scan_derivative[i]
            sum_ray, sum_depth, rays = 0.0, 0.0, 0
        elif scan_derivative[i] > jump:
            if on_cylinder and rays > 0:
                cylinder_list.append((sum_ray / rays, sum_depth / rays))
            on_cylinder = False
        elif scan[i] > min_dist:
            sum_ray += i
            sum_depth += scan[i]
            rays += 1
    return cylinder_list


if __name__ == '__main__':

    minimum_valid_distance = 20.0
    depth_jump = 100.0

    # Read the logfile which contains all scans.
    logfile = LegoLogfile()
    logfile.read("robot4_scan.txt")

    # Pick one scan.
    scan = logfile.scan_data[8]

    # Find cylinders.
    der = compute_derivative(scan, minimum_valid_distance)
    cylinders = find_cylinders(scan, der, depth_jump, minimum_valid_distance)

    # Plot results.
    plot(scan)
    plot(der)
    # grid()
    # yticks(range(-1000,3000,50))
    scatter([c[0] for c in cylinders], [c[1] for c in cylinders],
        c='r', s=200)
    show()
