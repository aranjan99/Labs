#!/usr/bin/env python3

'''
This is starter code for Lab 6 on Coordinate Frame transforms.

'''

import asyncio
import cozmo
import numpy
import math
from cozmo.util import degrees
from cozmo.util import Pose

def get_relative_pose(object_pose, reference_frame_pose):
	
	x,y,z = reference_frame_pose.position.x_y_z
	angle_z = reference_frame_pose.rotation.angle_z
	res_x,res_y,res_z = object_pose.position.x_y_z
	res_angle = object_pose.rotation.angle_z

	cos_angle = math.cos(angle_z.radians)
	sin_angle = math.sin(angle_z.radians)
	
	new_x = (res_x-x)*cos_angle + (res_y-y)*sin_angle

	new_y = -(res_x-x)*sin_angle + (res_y-y)*cos_angle

	new_z = res_z - z
	new_angle_z = res_angle - angle_z 
	return Pose(new_x, new_y, new_z, angle_z=new_angle_z)

def find_relative_cube_pose(robot: cozmo.robot.Robot):
	'''Looks for a cube while sitting still, prints the pose of the detected cube
	in world coordinate frame and relative to the robot coordinate frame.'''

	robot.move_lift(-3)
	robot.set_head_angle(degrees(0)).wait_for_completed()
	cube = None

	while True:
		try:
			cube = robot.world.wait_for_observed_light_cube(timeout=30)
			if cube:
				print("Robot pose: %s" % robot.pose)
				print("Cube pose: %s" % cube.pose)
				print("Cube pose in the robot coordinate frame: %s" % get_relative_pose(cube.pose, robot.pose))
		except asyncio.TimeoutError:
			print("Didn't find a cube")


if __name__ == '__main__':

	cozmo.run_program(find_relative_cube_pose)