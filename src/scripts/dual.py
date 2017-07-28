#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64MultiArray
from prettytable import PrettyTable
import math
import cv2

class ImageProcessing():
	def __init__(self):
		rospy.init_node('meteor_detection', anonymous=True)
		self.pub =  rospy.Publisher('panTilt', Float64MultiArray, queue_size=10)

	def image_processing(self):
		################################################################################################
		##### ENTER YOUR CODE HERE
		################################################################################################
		Track_tol = 30
		X_Step = math.pi/180*4
		kx=0.0011
		Y_Step = math.pi/180*4
		ky=0.0011  #0.001
		P = 0
		T = 0
		# HSV Limits
		greenLower = (0,0,255)
		greenUpper = (91,23,255)
		
		camera = cv2.VideoCapture(0)

		while True:
			# grab the current frame
			(grabbed, frame) = camera.read()

			blurred = cv2.GaussianBlur(frame, (9, 9), 0)
			hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
			height, width, channels = frame.shape
			im_x = width/2
			im_y = height/2

			# construct a mask for your color 

			mask = cv2.inRange(hsv, greenLower, greenUpper)
			mask = cv2.erode(mask, None, iterations=2)
			mask = cv2.dilate(mask, None, iterations=2)

			cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
			# only proceed if at least one contour was found
			if len(cnts) > 0:

				# find the largest contour
				c = max(cnts, key=cv2.contourArea)
				#for c in cnts:
				(x,y,w,h) = cv2.boundingRect(c)
				cv2.rectangle(frame,(x,y),(x+w,y+h),(0,255,0),2)

				X_des = x+(w/2)
				Y_des = y+(h/2)

				X_er = (X_des-im_x)
				Y_er = (Y_des-im_y)

				if abs(X_er) < Track_tol:
					P = 0
				else:
					P = -kx*X_er

				# elif X_er > 0:
				# 	P = kx*X_er #-X_Step

				# elif X_er < 0:
				# 	P =  -kx*X_er #X_Step

				if abs(Y_er) < Track_tol:
					T = 0
				else:
					T = -ky*Y_er

				# elif Y_er > 0:
				# 	T = -ky*Y_er #-Y_Step

				# elif Y_er < 0:
				# 	T = ky*Y_er#Y_Step
				print("Desired Corrections Pan: "+repr(P)+" and  Tilt: "+repr(T)+"\n")
				angles = Float64MultiArray()
				angles.data = [P, T]
				self.pub.publish(angles)
				# rospy.sleep(0.1)
			# # Did not detect any contours, then sweep
			# else:
			# 	self.pan_sweep()
			# 	self.tilt_sweep()
			# show the frame to our screen
			cv2.imshow("Frame", frame)
			cv2.imshow("Tresholded", mask)
			
			key = cv2.waitKey(1) & 0xFF
			# if the 'q' key is pressed, stop the loop
			if key == ord("q"):
				break
		# rospy.spin()


if __name__=='__main__':
  try:
    img = ImageProcessing()
    img.image_processing()
  except rospy.ROSInterruptException:
    pass
