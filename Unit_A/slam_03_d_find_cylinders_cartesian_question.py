# For each cylinder in the scan, find its cartesian coordinates,
# in the scanner's coordinate system.
# Write the result to a file which contains all cylinders, for all scans.
# 03_d_find_cylinders_cartesian
# Claus Brenner, 09 NOV 2012
from lego_robot import *
from math import sin, cos

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
# def find_cylinders(scan, scan_derivative, jump, min_dist):
# 	cylinder_list = []
# 	on_cylinder = False
# 	sum_ray, sum_depth, rays = 0.0, 0.0, 0
#
# 	# --->>> Insert here your previous solution from find_cylinders_question.py.
# 	cylinder_rays_list = []
# 	scan_list = []
# 	all_rays_list = []
#
# 	for i in xrange(len(scan_derivative)):
# 		# --->>> Insert your cylinder code here.
# 		# Whenever you find a cylinder, add a tuple
# 		# (average_ray, average_depth) to the cylinder_list.
# 		scan_list.append(scan[i])
# 		all_rays_list.append(i)
# 		if i < len(scan_derivative)-1:
# 			if scan[i+1] - scan[i]> jump or scan[i-1] - scan[i]> jump : #check for left and right limits of a cylinder
# 				if scan[i] < min_dist:#
# 					cylinder_rays_list.append(i+1)
# 				else:
# 					cylinder_rays_list.append(i)
#
# 	for j,k in enumerate(cylinder_rays_list):
# 		if j < len(cylinder_rays_list)-1 and j%2 ==0: #use the left and right limits of a cylinder in pairs
# 			start = cylinder_rays_list[j]
# 			end = cylinder_rays_list[j + 1] + 1 #use the value of the next index and add one to it to make search inclusive of both limits
# 			average_depth = sum(scan_list[start:end]) / float(len(scan_list[start:end]))
# 			average_ray = sum(range(start,end))/ float(len(range(start,end)))
# 			cylinder_list.append((average_ray,average_depth))
#
# 	return cylinder_list



# def find_cylinders(scan, scan_derivative, jump, min_dist):
#     cylinder_list = []
#     on_cylinder = False
#     direction = 'Left'
#     sum_ray, sum_depth, rays = 0.0, 0.0, 0
#     discard = False
#
#     for i in xrange(len(scan_derivative)):
#         # --->>> Insert your cylinder code here.
#         # Whenever you find a cylinder, add a tuple
#         # (average_ray, average_depth) to the cylinder_list.
#         current_der = scan_derivative[i]
#         if abs(current_der) > jump:
#             if on_cylinder and direction == 'Left':
#                 if current_der < 0: # Left again
#                     discard = True
#                 else:
#                     on_cylinder = False
#                     average_ray = sum_ray/rays
#                     average_depth = sum_depth/rays
#                     cylinder_list.append( (average_ray, average_depth) )
#                     sum_ray, sum_depth, rays = 0.0, 0.0, 0
#             if not on_cylinder and current_der < 0:
#                 on_cylinder = True
#                 direction = 'Left'
#         if scan[i] <= min_dist:
#             discard = True
#         if on_cylinder and scan[i] > min_dist:
#             rays += 1
#             sum_ray += i
#             sum_depth += scan[i]
#         if discard:
#             sum_ray, sum_depth, rays = 0.0, 0.0, 0
#             discard = False
#
#     return cylinder_list


def find_cylinders(scan, scan_derivative, jump, min_dist):
    cylinder_list = []
    on_cylinder = False
    sum_ray, sum_depth, rays = 0.0, 0.0, 0
    for i in xrange(len(scan_derivative)):
        if scan_derivative[i] < -jump:
            on_cylinder = True
            # print scan_derivative[i]
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



def compute_cartesian_coordinates(cylinders, cylinder_offset):
    result = []
    for c in cylinders:
        # --->>> Insert here the conversion from polar to Cartesian coordinates.
        # c is a tuple (beam_index, range).
        # For converting the beam index to an angle, use
        # LegoLogfile.beam_index_to_angle(beam_index)
		calibrated_range = c[1]+cylinder_offset
		angle_in_radians = LegoLogfile.beam_index_to_angle(c[0])
		#get x and y co-ordinates in robots frame given the range(hypotenuse)
		x = (calibrated_range * cos(angle_in_radians))
		y = (calibrated_range * sin(angle_in_radians))
		print x,y,angle_in_radians
		result.append( (x,y) ) # Replace this by your (x,y)
    return result
        

if __name__ == '__main__':

    minimum_valid_distance = 20.0
    depth_jump = 100.0
    cylinder_offset = 90.0

    # Read the logfile which contains all scans.
    logfile = LegoLogfile()
    logfile.read("robot4_scan.txt")

    # Write a result file containing all cylinder records.
    # Format is: D C x[in mm] y[in mm] ...
    # With zero or more points.
    # Note "D C" is also written for otherwise empty lines (no
    # cylinders in scan)
    out_file = file("cylinders.txt", "w")
    for scan in logfile.scan_data:
        # Find cylinders.
        der = compute_derivative(scan, minimum_valid_distance)
        cylinders = find_cylinders(scan, der, depth_jump,minimum_valid_distance)
        cartesian_cylinders = compute_cartesian_coordinates(cylinders,cylinder_offset)

        # Write to file.
        print >> out_file, "D C",
        for c in cartesian_cylinders:
            print >> out_file, "%.1f %.1f" % c,
        print >> out_file
    out_file.close()
