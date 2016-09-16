#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import time

#GLOBAL VARIABLES
#################
r = None
cmd_vel = None
#################


def forward (direction, speed, duration):
	print 'This should go forward at ' + speed + ' meters for second, during ' + duration + ' seconds.'
	move_cmd = Twist()
	move_cmd.angular.z=0
	x=0.01
	while x < float(speed):
		move_cmd.linear.x = float(direction)*float(x)
		x += 0.01
		cmd_vel.publish(move_cmd)
		rospy.sleep(0.1)
	timeout = time.time() + float(duration)
	move_cmd.linear.x = float(direction)*float(speed)
	while True:
		if time.time() > timeout:
			break
		else:
			cmd_vel.publish(move_cmd)
			r.sleep()
	while x > 0:
		move_cmd.linear.x = float(direction)*float(x)
		x -= 0.01
		cmd_vel.publish(move_cmd)
		rospy.sleep(0.1)
	cmd_vel.publish(Twist())


def rotate (direction, speed, duration):
	print 'This should rotate clockwise at ' + speed + ' radians for second, during ' + duration + ' seconds.'
	move_cmd = Twist()
	move_cmd.linear.x = 0
	move_cmd.angular.z= float(direction)*float(speed)
	timeout = time.time() + float(duration)
	while True:
		if time.time() > timeout:
			break
		else:
			cmd_vel.publish(move_cmd)
			r.sleep()
	cmd_vel.publish(Twist())


def file__parse(file_path):
	f = open(file_path)
	for line in f:
		line_arr = line.split()
		if line_arr[0] == 'move':
			forward(line_arr[1], line_arr[2], line_arr[3])
		elif line_arr[0] == 'rotate':
			rotate(line_arr[1], line_arr[2], line_arr[3])
		else:
			print "The entered command: " + str(line_arr[:])+ " is illegal."
		rospy.sleep(1)
		


def shutdown():
	print "Stop TurtleBot"
# a default Twist has linear.x of 0 and angular.z of 0.  So it'll stop TurtleBot
	cmd_vel.publish(Twist())
# sleep just makes sure TurtleBot receives the stop command prior to shutting down the script
	rospy.sleep(1)


if __name__ == "__main__":
	rospy.init_node('movement', anonymous=False)
	cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)
	r = rospy.Rate(10)
#	rospy.on_shutdown(shutdown())
	file__parse('corridor2.txt')
