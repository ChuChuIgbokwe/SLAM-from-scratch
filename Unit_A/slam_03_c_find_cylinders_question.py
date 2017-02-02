# For each cylinder in the scan, find its ray and depth.
# 03_c_find_cylinders
# Claus Brenner, 09 NOV 2012

from pylab import *
from lego_robot import *
#got it working 31/03/2016 by 12.05am
# Find the derivative in scan data, ignoring invalid measurements.
def compute_derivative(scan, min_dist):
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

# For each area between a left falling edge and a right rising edge,
# determine the average ray number and the average depth.
def find_cylinders(scan, scan_derivative, jump, min_dist):
	'''
	This function finds the average depth of a cylinder while ignoring extraneous values.
	:param scan:
	:param scan_derivative:
	:param jump:
	:param min_dist:
	:return:
	'''
	cylinder_list = []
	# on_cylinder = False
	# sum_ray, sum_depth, rays = 0.0, 0.0, 0

	cylinder_rays_list = []
	scan_list = []
	all_rays_list = []

	for i in xrange(len(scan_derivative)):
		# --->>> Insert your cylinder code here.
		# Whenever you find a cylinder, add a tuple
		# (average_ray, average_depth) to the cylinder_list.
		scan_list.append(scan[i])
		all_rays_list.append(i)
		if i < len(scan_derivative)-1:
			if scan[i+1] - scan[i]> jump or scan[i-1] - scan[i]> jump : #check for left and right limits of a cylinder
				if scan[i] < min_dist:#get rid of extraneous values, append the value after it to cylinder_rays_list
					cylinder_rays_list.append(i+1)
				else:
					cylinder_rays_list.append(i)

	for j,k in enumerate(cylinder_rays_list):
		# use the left and right limits of a cylinder in pairs. This method guarantees you use indexes 0-1, 2-3, 4-5, etc
		#together while stopping the check at the last index
		if j < len(cylinder_rays_list)-1 and j%2 ==0:
			start = cylinder_rays_list[j]
			end = cylinder_rays_list[j + 1] + 1 #use the value of the next index and add one to it to make search inclusive of both limits
			average_depth = sum(scan_list[start:end]) / float(len(scan_list[start:end]))
			average_ray = sum(range(start,end))/ float(len(range(start,end)))
			cylinder_list.append((average_ray,average_depth))

	# Just for fun, I'll output some cylinders.
	# Replace this by your code.
	# 	if i % 100 == 0:
	# 		cylinder_list.append( (i, scan[i]) )

	return cylinder_list

if __name__ == '__main__':

    minimum_valid_distance = 20.0
    depth_jump = 250.0

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
    scatter([c[0] for c in cylinders], [c[1] for c in cylinders],
        c='r', s=200)
    show()
