# Implement the second move model for the Lego robot.
# The difference to the first implementation is:
# - added a scanner displacement
# - added a different start pose (measured in the real world)
# - result is now output to a file, as "F" ("filtered") records.
#
# 02_b_filter_motor_file
# Claus Brenner, 09 NOV 2012
from math import sin, cos, pi
from lego_robot import *
from pylab import *

# This function takes the old (x, y, heading) pose and the motor ticks
# (ticks_left, ticks_right) and returns the new (x, y, heading).
def filter_step(old_pose, motor_ticks, ticks_to_mm, robot_width,
                scanner_displacement):

    # Find out if there is a turn at all.
    if motor_ticks[0] == motor_ticks[1]:
        # No turn. Just drive straight.

        # --->>> Use your previous implementation.
        # Think about if you need to modify your old code due to the
        # scanner displacement?
        x,y,theta = old_pose
        x = x + motor_ticks[0] * ticks_to_mm * cos(theta)
        y = y + motor_ticks[0] * ticks_to_mm * sin(theta)
        # theta = old_pose[2]
        return (x, y, theta)

    else:
        # Turn. Compute alpha, R, etc.
        # old_theta = old_pose[2]
        # old_x = old_pose[0] - cos(old_theta) * scanner_displacement
        # old_y = old_pose[1] - sin(old_theta) * scanner_displacement
        #
        # l = motor_ticks[0] * ticks_to_mm
        # r = motor_ticks[1] * ticks_to_mm
        # alpha = (r - l) / robot_width
        # R = l / alpha
        # theta = (old_theta + alpha) % (2.0 * pi)
        #
        # # --->>> Modify your previous implementation.
        # # First modify the the old pose to get the center (because the
        # #   old pose is the LiDAR's pose, not the robot's center pose).
        # # Second, execute your old code, which implements the motion model
        # #   for the center of the robot.
        #
        # Cx = old_x - (R + robot_width / 2.0) * sin(old_theta)
        # Cy = old_y - (R + robot_width / 2.0) * (-cos(old_theta))
        #
        # x = Cx + (R + robot_width / 2.0) * sin(theta)
        # y = Cy + (R + robot_width / 2.0) * (-cos(theta))
        #
        # # Third, modify the result to get back the LiDAR pose from
        # #   your computed center. This is the value you have to return.
        # x += cos(theta) * scanner_displacement
        # y += sin(theta) * scanner_displacement
        #
        # return (x, y, theta)

        # Turn. Compute alpha, R, etc.
        # Modification: subtract offset to compute center.
        old_x = old_pose[0]
        old_y = old_pose[1]
        old_theta = old_pose[2]

        old_x -= cos(old_theta) * scanner_displacement
        old_y -= sin(old_theta) * scanner_displacement

        l = motor_ticks[0] * ticks_to_mm
        r = motor_ticks[1] * ticks_to_mm
        alpha = (r - l) / robot_width
        R = l / alpha
        theta = (old_theta + alpha) % (2.0 * pi)

        c_x = old_x - (R + robot_width / 2.0) * sin(old_theta)
        c_y = old_y - (R + robot_width / 2.0) * (-cos(old_theta))

        x = c_x + (R + robot_width / 2.0) * sin(theta)
        y = c_y + (R + robot_width / 2.0) * (-cos(theta))

        x += cos(theta) * scanner_displacement
        y += sin(theta) * scanner_displacement
        # --->>> Modify your previous implementation.
        # First modify the the old pose to get the center (because the
        #   old pose is the LiDAR's pose, not the robot's center pose).
        # Second, execute your old code, which implements the motion model
        #   for the center of the robot.
        # Third, modify the result to get back the LiDAR pose from
        #   your computed center. This is the value you have to return.

        return (x, y, theta)




if __name__ == '__main__':
    # Empirically derived distance between scanner and assumed
    # center of robot.
    scanner_displacement = 30.0

    # Empirically derived conversion from ticks to mm.
    ticks_to_mm = 0.349

    # Measured width of the robot (wheel gauge), in mm.
    robot_width =  173.0 #150.0

    # Measured start position.
    pose = (1850.0, 1897.0, 213.0 / 180.0 * pi)

    # Read data.
    logfile = LegoLogfile()
    logfile.read("robot4_motors.txt")

    # Loop over all motor tick records generate filtered position list.
    filtered = []
    for ticks in logfile.motor_ticks:
        pose = filter_step(pose, ticks, ticks_to_mm, robot_width,
                           scanner_displacement)
        filtered.append(pose)

	 # Draw result.
    for pose in filtered:
        print pose
    plot([p[0] for p in filtered], [p[1] for p in filtered], 'bo')
    show()

    # Write all filtered positions to file.
    f = open("poses_from_ticks.txt", "w")
    for pose in filtered:
        print >> f, "F %f %f %f" % pose
    f.close()
