#!/usr/bin/env python3

import cv2
import sys
import copy
import matplotlib
import skimage
from skimage import feature, color
from matplotlib import pyplot as plt
from skimage.measure import ransac, LineModelND
from skimage.measure import CircleModel
import numpy as np
try:
	from PIL import Image, ImageDraw, ImageFont
except ImportError:
	sys.exit('install Pillow to run this code')


def find_ball(opencv_image, debug=False):
	"""Find the ball in an image.
		
		Arguments:
		opencv_image -- the image
		debug -- an optional argument which can be used to control whether
				debugging information is displayed.
		
		Returns [x, y, radius] of the ball, and [0,0,0] or None if no ball is found.
	"""

	ball = None	

	opencv_image = cv2.bilateralFilter(opencv_image,7,90,90)
	circles = cv2.HoughCircles(opencv_image, cv2.cv.CV_HOUGH_GRADIENT, 2, 1000, maxRadius = 100 )
	if circles is not None:
		ball = circles[0][0, :]
	else:
		circles = cv2.HoughCircles(opencv_image, cv2.cv.CV_HOUGH_GRADIENT,4, 1000, maxRadius = 100)
		if circles is not None:
			ball = circles[0][0, :]		
	if debug and circles is not None:
		display_circles(opencv_image, circles[0], ball)
	return ball


def display_circles(opencv_image, circles, best=None):
	"""Display a copy of the image with superimposed circles.
		
	   Provided for debugging purposes, feel free to edit as needed.
	   
	   Arguments:
		opencv_image -- the image
		circles -- list of circles, each specified as [x,y,radius]
		best -- an optional argument which may specify a single circle that will
				be drawn in a different color.  Meant to be used to help show which
				circle is ranked as best if there are multiple candidates.
		
	"""
	#make a copy of the image to draw on
	circle_image = copy.deepcopy(opencv_image)
	circle_image = cv2.cvtColor(circle_image, cv2.COLOR_GRAY2RGB, circle_image)
	
	for c in circles:
		# draw the outer circle
		cv2.circle(circle_image,(c[0],c[1]),c[2],(255,255,0),2)
		# draw the center of the circle
		cv2.circle(circle_image,(c[0],c[1]),2,(0,255,255),3) 
		# write coords          
	
	#highlight the best circle in a different color
	if best is not None:
		# draw the outer circle
		cv2.circle(circle_image,(best[0],best[1]),best[2],(0,0,255),2)
		# draw the center of the circle
		cv2.circle(circle_image,(best[0],best[1]),2,(0,0,255),3) 
		# write coords          
		
	
	#display the image
	pil_image = Image.fromarray(circle_image)
	pil_image.show()    
	  
if __name__ == "__main__":
	pass
