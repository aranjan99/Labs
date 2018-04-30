#!/usr/bin/env python3

'''
Stater code for Lab 7.
'''

import cozmo
from cozmo.util import degrees, Angle, Pose, distance_mm, speed_mmps
import math
import time

# Wrappers for existing Cozmo navigation functions

def cozmo_drive_straight(robot, dist, speed):
	"""Drives the robot straight.
		Arguments:
		robot -- the Cozmo robot instance passed to the function
		dist -- Desired distance of the movement in millimeters
		speed -- Desired speed of the movement in millimeters per second
	"""
	robot.drive_straight(distance_mm(dist), speed_mmps(speed)).wait_for_completed()

def cozmo_turn_in_place(robot, angle, speed):
	"""Rotates the robot in place.
		Arguments:
		robot -- the Cozmo robot instance passed to the function
		angle -- Desired distance of the movement in degrees
		speed -- Desired speed of the movement in degrees per second
	"""
	robot.turn_in_place(degrees(angle), speed=degrees(speed)).wait_for_completed()

def cozmo_go_to_pose(robot, x, y, angle_z):
	"""Moves the robot to a pose relative to its current pose.
		Arguments:
		robot -- the Cozmo robot instance passed to the function
		x,y -- Desired position of the robot in millimeters
		angle_z -- Desired rotation of the robot around the vertical axis in degrees
	"""
	robot.go_to_pose(Pose(x, y, 0, angle_z=degrees(angle_z)), relative_to_robot=True).wait_for_completed()

# Functions to be defined as part of the labs

def get_front_wheel_radius():
	"""Returns the radius of the Cozmo robot's front wheel in millimeters."""
	# used drive straight function with experimentation to adjust the distance and let cozmo drive 2 full rotations.
	# Used the distance traveled to calculate radius as 2 * PI * Radiis * Number of rotations = Distance traveled
	return 14;

def get_distance_between_wheels(): 
	"""Returns the distance between the wheels of the Cozmo robot in millimeters."""
	# ####
	# Calculated the time taken for robot to revolve once around z axis with fixed velocity using drive_wheels method, and got the distance as 
	# speed * time/ (2 * 3.14)
	return 50;

def rotate_front_wheel(robot, angle_deg):
	"""Rotates the front wheel of the robot by a desired angle.
		Arguments:
		robot -- the Cozmo robot instance passed to the function
		angle_deg -- Desired rotation of the wheel in degrees
	"""
	radius = get_front_wheel_radius();
	dist = angle_deg * 3.14 * radius / 180;
	robot.drive_straight(distance_mm(dist), speed_mmps(5)).wait_for_completed()


def my_drive_straight(robot, dist, speed):
	"""Drives the robot straight.
		Arguments:
		robot -- the Cozmo robot instance passed to the function
		dist -- Desired distance of the movement in millimeters
		speed -- Desired speed of the movement in millimeters per second
	"""
	#print(speed, dist, dist * 1.0/ speed )
	robot.drive_wheels(speed, speed, duration = dist * 1.0/ speed )
	time.sleep(dist * 1.0/ speed)

def my_turn_in_place(robot, angle, speed):
	"""Rotates the robot in place.
		Arguments:
		robot -- the Cozmo robot instance passed to the function
		angle -- Desired distance of the movement in degrees
		speed -- Desired speed of the movement in degrees per second
	"""
	
	#print(speed, angle,  angle * 6.1 / 360)
	distanceBetweenWheels = get_distance_between_wheels()
	distanceToTravel = angle * 3.14 * distanceBetweenWheels/ 180
	linearSpeed = speed * 3.14 * distanceBetweenWheels/180
	time = distanceToTravel / linearSpeed 
	if(angle < 0):
		distanceToTravel = -distanceToTravel
		linearSpeed = -linearSpeed
		time = -time
	robot.drive_wheels(linearSpeed, -linearSpeed, duration = time) 
	

def my_go_to_pose1(robot, res_x,res_y, angle_z):
	"""Moves the robot to a pose relative to its current pose.
		Arguments:
		robot -- the Cozmo robot instance passed to the function
		x,y -- Desired position of the robot in millimeters
		angle_z -- Desired rotation of the robot around the vertical axis in degrees
	"""
	x,y,z = robot.pose.position.x_y_z
	angle_r = robot.pose.rotation.angle_z
	res_angle = angle_z

	cos_angle = math.cos(angle_r.radians)
	sin_angle = math.sin(angle_r.radians)
	
	new_x = (res_x-x)*cos_angle + (res_y-y)*sin_angle

	new_y = -(res_x-x)*sin_angle + (res_y-y)*cos_angle

	print(new_x, new_y)
	new_angle_z = res_angle - (angle_r.radians*180/3.14)

	print("angle" , math.atan(new_x/new_y) * 180/ 3.14)
	my_turn_in_place(robot, math.atan(new_x/new_y) * 180/ 3.14, 30);
	my_drive_straight(robot, math.sqrt(new_x*new_x + new_y*new_y), 50);
	print("angle" , new_angle_z)
	my_turn_in_place(robot, new_angle_z, 30);

def my_go_to_pose2(robot, res_x, res_y, res_angle):
	"""Moves the robot to a pose relative to its current pose.
		Arguments:
		robot -- the Cozmo robot instance passed to the function
		x,y -- Desired position of the robot in millimeters
		angle_z -- Desired rotation of the robot around the vertical axis in degrees
	"""
	x,y,z = robot.pose.position.x_y_z
	angle_r = robot.pose.rotation.angle_z		

	rho = math.sqrt((res_x - x) * (res_x - x) + (res_y - y) * (res_y - y))
	alpha = - angle_r.radians + math.atan((res_y - y)/(res_x - x))
	eta = (res_angle*3.14/180) - angle_r.radians
				
	while rho > 10 or alpha + eta > 10*3.14/180: 
		x,y,z = robot.pose.position.x_y_z
		angle_r = robot.pose.rotation.angle_z		

		rho = math.sqrt((res_x - x) * (res_x - x) + (res_y - y) * (res_y - y))
		alpha = - angle_r.radians + math.atan((res_y - y)/(res_x - x))
		eta = (res_angle*3.14/180) - angle_r.radians
		X = rho
		Theta = alpha + eta
		
		print ((2*X - (Theta * get_distance_between_wheels()))/(2.0*get_front_wheel_radius()))
		print ((2*X + (Theta * get_distance_between_wheels()))/(2.0*get_front_wheel_radius()))
		
		robot.drive_wheels((2*X - (Theta * get_distance_between_wheels()))/(2.0*get_front_wheel_radius()), 
		(2*X + (Theta * get_distance_between_wheels()))/(2.0*get_front_wheel_radius()), duration = 1) 		

		print("Robot pose: %s" % robot.pose)

def my_go_to_pose3(robot, res_x, res_y, angle_z):
	"""Moves the robot to a pose relative to its current pose.
		Arguments:
		robot -- the Cozmo robot instance passed to the function
		x,y -- Desired position of the robot in millimeters
		angle_z -- Desired rotation of the robot around the vertical axis in degrees
	"""
	x,y,z = robot.pose.position.x_y_z
	angle_r = robot.pose.rotation.angle_z
	res_angle = angle_z

	cos_angle = math.cos(angle_r.radians)
	sin_angle = math.sin(angle_r.radians)
	
	new_x = (res_x-x)*cos_angle + (res_y-y)*sin_angle

	new_y = -(res_x-x)*sin_angle + (res_y-y)*cos_angle

	print("Robot final pose: %s" % new_y, new_x)
	if(new_y < 0):
		print("Robot turn by angle: " + str(math.atan(new_x/new_y) * 180/ 3.14))
		my_turn_in_place(robot, math.atan(new_x/new_y) * 180/ 3.14, 30);

	rho = math.sqrt((res_x - x) * (res_x - x) + (res_y - y) * (res_y - y))
	alpha = - angle_r.radians + math.atan((res_y - y)/(res_x - x))
	eta = (res_angle*3.14/180) - angle_r.radians
				
	while rho > 10 or alpha + eta > 10*3.14/180: 
		x,y,z = robot.pose.position.x_y_z
		angle_r = robot.pose.rotation.angle_z		

		rho = math.sqrt((res_x - x) * (res_x - x) + (res_y - y) * (res_y - y))
		alpha = - angle_r.radians + math.atan((res_y - y)/(res_x - x))
		eta = (res_angle*3.14/180) - angle_r.radians
		X = rho
		Theta = alpha + eta
		
		print ((2*X - (Theta * get_distance_between_wheels()))/(2.0*get_front_wheel_radius()))
		print ((2*X + (Theta * get_distance_between_wheels()))/(2.0*get_front_wheel_radius()))
		
		robot.drive_wheels((2*X - (Theta * get_distance_between_wheels()))/(2.0*get_front_wheel_radius()), 
		(2*X + (Theta * get_distance_between_wheels()))/(2.0*get_front_wheel_radius()), duration = 1) 		

		print("Robot pose: %s" % robot.pose)


def run(robot: cozmo.robot.Robot):

	print("***** Front wheel radius: " + str(get_front_wheel_radius()))
	print("***** Distance between wheels: " + str(get_distance_between_wheels()))

	## Example tests of the functions

	print("Robot pose: %s" % robot.pose)
				
	my_go_to_pose3(robot, -269, 127, 20)

	print("Robot pose: %s" % robot.pose)
	

if __name__ == '__main__':

	cozmo.run_program(run)