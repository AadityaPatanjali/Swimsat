#!/usr/bin/env python

import sys
import select
import rospy
import moveit_commander
import moveit_msgs.msg
from std_msgs.msg import Float64MultiArray
from prettytable import PrettyTable
from arbotix_python.joints import *
import math
import cv2
import copy
from PID import PID
import time

class ArmMovement():
	def __init__(self):
		moveit_commander.roscpp_initialize(sys.argv)
		rospy.init_node('move_group_python', anonymous=True)
		self.robot = moveit_commander.RobotCommander()
		self.scene = moveit_commander.PlanningSceneInterface()
		self.group = moveit_commander.MoveGroupCommander("arm")
		# self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory,queue_size=10)
		joint_defaults = getJointsFromURDF()
		joints = rospy.get_param('/arbotix/joints', dict())
		self.joint_limits = list()
		# Get joint limits
		for name in sorted(joints.keys()):
			# pull angles
			min_angle, max_angle = getJointLimits(name, joint_defaults)
			self.joint_limits.append({'name':name, 'min_angle':min_angle*math.pi/180.0, 'max_angle':max_angle*math.pi/180.0})
		self.pan_index  = 0
		self.tilt_index = 3
		self.disturbance_index = 2
		self.disturbance_down = False
		self.disturbance_step = math.pi/180.0*0.05
		self.set_disturbance_limits(width=0.05)
		self.pan_sweep_step = math.pi/180.0*10.0
		self.tilt_sweep_step = math.pi/180.0*10.0
		self.pan_sweep_max = 40.0
		self.pan_sweep_min = -40.0
		self.tilt_sweep_max = 9.0
		self.tilt_sweep_min = -40.0
		self.pan_left =True
		self.tilt_up = True
		self.set_homing_position()

	def __del__(self):
		# self.set_homing_position()		
		self.group.set_joint_value_target([0,0,0,0,0])
		self.group.go(wait=True)
		## When finished shut down moveit_commander.
		moveit_commander.roscpp_shutdown()

	def subscribe(self):
		rospy.Subscriber('panTilt',Float64MultiArray,self.move_joints_callback)


	def set_pan_index(self,pan_index):
		self.pan_index = pan_index

	def get_pan_index(self):
		return self.pan_index

	def set_tilt_index(self,tilt_index):
		self.tilt_index = tilt_index

	def get_tilt_index(self):
		return self.tilt_index

	def set_pan_sweep_step(self,pan_sweep_step):
		self.pan_sweep_step = pan_sweep_step

	def set_tilt_sweep_step(self,tilt_sweep_step):
		self.tilt_sweep_step = tilt_sweep_step

	def get_pan_sweep_step(self):
		return self.pan_sweep_step

	def get_tilt_sweep_step(self):
		return self.tilt_sweep_step

	def set_pan_sweep_max(self,pan_sweep_max):
		self.pan_sweep_max = pan_sweep_max

	def set_tilt_sweep_max(self,tilt_sweep_max):
		self.tilt_sweep_max = tilt_sweep_max

	def set_pan_sweep_min(self,pan_sweep_min):
		self.pan_sweep_min = pan_sweep_min

	def set_tilt_sweep_min(self,tilt_sweep_min):
		self.tilt_sweep_min = tilt_sweep_min

	def get_pan_sweep_max(self):
		return self.pan_sweep_max

	def get_tilt_sweep_max(self):
		return self.tilt_sweep_max

	def get_pan_sweep_min(self):
		return self.pan_sweep_min

	def get_tilt_sweep_min(self):
		return self.tilt_sweep_min

	def get_current_joint_values(self,degrees = False):
		group_variable_values = self.group.get_current_joint_values()
		if not degrees:
			return group_variable_values
		else:
			return [ele*180.0/math.pi for ele in group_variable_values]

	def set_disturbance_limits(self, width = 5.0):
		angle = self.get_current_joint_values(degrees=True)[self.disturbance_index]
		self.disturbance_max = math.pi/180.0*(angle + width)
		self.disturbance_min = math.pi/180.0*(angle - width)

	def set_homing_position(self,initial=True):
		# # Previous angles
		# angles = [0.0,60.0,-81.0,-60.0,0.0]
		# # 642 Setup 1 angles
		# angles = [0.0,28.0,-32.0,-70.0,0.0]		
		# # SmallSat Setup backup
		# angles = [-0.20,17.72,-31.64,-66.50,0.0]
		angles = [-0.20,17.72,-31.64,-66.50,0.0]
		# if not initial:
		# 	group_variable_values = self.get_current_joint_values(degrees = True)
		# 	angles[3] = group_variable_values[3]

		group_variable_values = [ele*math.pi/180.0 for ele in angles]
		self.group.set_joint_value_target(group_variable_values)
		self.group.go(wait=initial)

	def get_max_limits(self,degrees=False):
		if degrees:
			return [self.joint_limits[i].get('max_angle')*180.0/math.pi*100 for i in range(0,5)]
		else:
			return [self.joint_limits[i].get('max_angle') for i in range(0,5)]

	def get_min_limits(self,degrees=False):
		if degrees:
			return [self.joint_limits[i].get('min_angle')*180.0/math.pi*100 for i in range(0,5)]
		else:
			return [self.joint_limits[i].get('min_angle') for i in range(0,5)]

	def get_pan_tilt_min(self,degrees=False):
		if degrees:
			min = self.get_min_limits(degrees=True)
			return [ min[self.pan_index], min[self.tilt_index] ] 
		else:
			min = self.get_min_limits()
			return [ min[self.pan_index], min[self.tilt_index]  ]

	def get_pan_tilt_max(self,degrees=False):
		if degrees:
			max = self.get_max_limits(degrees=True)
			return [ max[self.pan_index], max[self.tilt_index] ] 
		else:
			max = self.get_max_limits()
			return [ max[self.pan_index], max[self.tilt_index]  ]

	def pan_sweep(self):
		group_variable_values = self.group.get_current_joint_values()
		if self.pan_left:
			move_to = group_variable_values[self.pan_index]+self.pan_sweep_step
			if  move_to > self.pan_sweep_max:
				self.pan_left = False
			else:
				group_variable_values[self.pan_index] = move_to
				try:
					self.group.set_joint_value_target(group_variable_values)
					# plan2 = group.plan()
					self.group.go(wait=False)
				except:
					pass
		else:
			move_to = group_variable_values[self.pan_index]-self.pan_sweep_step
			if  move_to < self.pan_sweep_min:
				self.pan_left = True
			else:
				group_variable_values[self.pan_index] = move_to
				try:
					self.group.set_joint_value_target(group_variable_values)
					# plan2 = group.plan()
					self.group.go(wait=False)
				except:
					pass

	def tilt_sweep(self):
		group_variable_values = self.group.get_current_joint_values()
		if self.tilt_up:
			move_to = group_variable_values[self.tilt_index]+self.tilt_sweep_step
			if  move_to > self.tilt_sweep_max:
				self.tilt_up = False
			else:
				group_variable_values[self.tilt_index] = move_to
				try:
					self.group.set_joint_value_target(group_variable_values)
					# plan2 = group.plan()
					self.group.go(wait=False)
				except:
					pass
		else:
			move_to = group_variable_values[self.tilt_index]-self.tilt_sweep_step
			if  move_to < self.tilt_sweep_min:
				self.tilt_up = True
			else:
				group_variable_values[self.tilt_index] = move_to
				try:
					self.group.set_joint_value_target(group_variable_values)
					# plan2 = group.plan()
					self.group.go(wait=False)
				except:
					pass

	def move_joints_callback(self,data):
		[pan,tilt] = data.data
		# print "== Pan and Tilt ",(pan,tilt)
		group_variable_values = self.group.get_current_joint_values()
		group_variable_values_copy = copy.deepcopy(group_variable_values)
		group_variable_values[self.pan_index] += pan
		group_variable_values[self.tilt_index] += tilt
		try:
			self.group.set_joint_value_target(group_variable_values)
			# plan2 = group.plan()
			self.group.go(wait=False)
		except:
			# self.group.set_joint_value_target(group_variable_values_copy)
			# # plan2 = group.plan()
			# self.group.go(wait=False)
			print "No plan found. Setting to original value"


	def move_arm(self,pan,tilt):
		group_variable_values = self.group.get_current_joint_values()
		group_variable_values_copy = copy.deepcopy(group_variable_values)
		group_variable_values[self.pan_index] += pan
		group_variable_values[self.tilt_index] += tilt
		try:
			self.group.set_joint_value_target(group_variable_values)
			# plan2 = group.plan()
			self.group.go(wait=False)
		except:
			# self.group.set_joint_value_target(group_variable_values_copy)
			# # plan2 = group.plan()
			# self.group.go(wait=False)
			print "No plan found. Setting to original value"

	def introduce_disturbance(self,disturbance_value,waiting):
		group_variable_values = self.group.get_current_joint_values()
		group_variable_values_copy = copy.deepcopy(group_variable_values)
		final = group_variable_values[self.disturbance_index] + disturbance_value
		if final > self.disturbance_max:
			group_variable_values[self.disturbance_index] = self.disturbance_max
		elif final < self.disturbance_min:
			group_variable_values[self.disturbance_index] = self.disturbance_min
		else: group_variable_values[self.disturbance_index] = final
		try:
			self.group.set_joint_value_target(group_variable_values)
			# plan2 = group.plan()
			self.group.go(wait=waiting)
		except:
			# self.group.set_joint_value_target(group_variable_values_copy)
			# # plan2 = group.plan()
			# self.group.go(wait=waiting)
			print "No plan found. Setting to original value"
		# error = math.pi/180.0*0.2
		# while True:
		# 	group_variable_values = self.group.get_current_joint_values()
		# 	group_variable_values_copy = self.group.get_current_joint_values()
		# 	group_variable_values[self.disturbance_index] += self.disturbance_step
		# 	try:
		# 		self.group.set_joint_value_target(group_variable_values)
		# 		# plan2 = group.plan()
		# 		self.group.go(wait=waiting)
		# 	except:
		# 		self.group.set_joint_value_target(group_variable_values_copy)
		# 		# plan2 = group.plan()
		# 		self.group.go(wait=waiting)
		# 		print "No plan found. Setting to original value"
		# 	if group_variable_values[self.disturbance_index] < (final+error) and group_variable_values[self.disturbance_index] > (final-error):
		# 		break


	def disturb(self,wait):
		if self.disturbance_down:
			self.introduce_disturbance(self.disturbance_step,wait)
			self.disturbance_down = False
		else:
			self.introduce_disturbance(-self.disturbance_step,wait)
			self.disturbance_down = True

	def __move_by_pose_orientation__(self,orientation_w=0,orientation_x=0,orientation_y=0,orientation_z=1.0):
		print "============ Generating plan"
		current_pose = self.group.get_current_pose()
		pose_target = current_pose.pose
		print "Current pose: %s" %current_pose.pose
		pose_target.orientation.w = orientation_w
		pose_target.orientation.x = orientation_x
		pose_target.orientation.y = orientation_y
		pose_target.orientation.z = orientation_z
		# pose_target.position.x = 0.65
		# pose_target.position.y = -1.0
		# pose_target.position.z = 1.0
		self.group.set_pose_target(pose_target)
		#group.set_random_target()
		self.group.go(wait=True)

	def __test_joint_movement__(self):
		while True:
			group_variable_values = self.group.get_current_joint_values()
			print "Max Angles", self.get_max_limits(degrees=True)
			print "Min Angles", self.get_min_limits(degrees=True)
			print "Current joint angles:", [ele*180.0/math.pi for ele in group_variable_values]
			try:
				angle = float(input("Enter angle to turn: "))
				index = int(input("Enter index: "))
				print "=========== Moving by ", angle, " degrees"
				group_variable_values[index] += angle*math.pi/180.0
				print "New angles will be:", [ele*180.0/math.pi for ele in group_variable_values]
				self.group.set_joint_value_target(group_variable_values)
				self.group.go(wait=True)
			except:
				print "Exiting!"
				break

	def __test_single_joint_movement__(self,angle,index):
		group_variable_values = self.group.get_current_joint_values()
		group_variable_values[index] = angle*math.pi/180.0
		self.group.set_joint_value_target(group_variable_values)
		self.group.go(wait=True)

	def __test_echo_current_joint_values__(self):
		# camera = cv2.VideoCapture(1)
		t = PrettyTable(['0','1','2','3','4'])
		while True:
			# (grabbed, frame) = camera.read()
			# cv2.imshow("Frame", frame)
			group_variable_values = self.group.get_current_joint_values()
			t.add_row([ele*180.0/math.pi for ele in group_variable_values])
			print t
			# print "Current joint angles:", [ele*180.0/math.pi*100 for ele in group_variable_values]
			if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
				line = raw_input()
				break
				
	def __test_print_info__(self):
		## We can get the name of the reference frame for this robot
		print "============ Reference frame: %s" % group.get_planning_frame()
		## We can also print the name of the end-effector link for this group
		print "============ Reference frame: %s" % group.get_end_effector_link()
		## We can get a list of all the groups in the robot
		print "============ Robot Groups:"
		print self.robot.get_group_names()
		## Sometimes for debugging it is useful to print the entire state of the
		## robot.
		print "============ Printing robot state"
		print self.robot.get_current_state()

	def image_processing(self):
		################################################################################################
		##### ENTER YOUR CODE HERE
		################################################################################################
		
		
		Track_tol = 30

		kpx = 0.0005 #0.00063 #0.0006 #0.00057  55  53
		kdx = 0.00008  #0.00006/7

		kpy = 0.0003  #0.001 #0.0007 0.00045 0.00043/4/35 33
		kdy = 0.00009  #0.0007

		P = 0
		T = 0

		dt = 0
		Vx = 0
		Vy = 0

		x_pre = 0
		y_pre = 0

		
		homing_count = 0
		max_homing_count = 20
		# HSV Limits
		# Bottle Cap
		#greenLower = (44,127,48)
		#greenUpper = (112,255,255)

		# # Meteor green
		# greenLower = (0,0,255)
		# greenUpper = (162,32,255)

		# # USU 201A Meteor Cap night
		# greenLower = (0,43,147)
		# greenUpper = (26,215,255)

		# USU 201A Meteor Cap day
		#greenLower = (0,69,230)
		#greenUpper = (50,122,255)

		# # SmallSat Poster booth backup
		# greenLower = (12,37,245)
		# greenUpper = (34,101,255)

		# SmallSat Poster booth
		greenLower = (12,37,245)
		greenUpper = (34,101,255)
		
		
		camera = cv2.VideoCapture(1)


		while True:
			t = time.time()
			# self.disturb(False)
			# grab the current frame
			(grabbed, frame) = camera.read()

			blurred = cv2.GaussianBlur(frame, (9, 9), 0)
			hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
			height, width, channels = frame.shape
			im_x = width/2
			im_y = height/2
			mar_lin = 5
			font = cv2.FONT_HERSHEY_SIMPLEX
			cv2.putText(frame,'Incomming Feed',(width-240,height-5), font, 0.9,(255,255,255),2)
			cv2.line(frame,(im_x-mar_lin,im_y-mar_lin) ,(im_x+mar_lin,im_y+mar_lin),(0,0,255),4)
			cv2.line(frame,(im_x-mar_lin,im_y+mar_lin) ,(im_x+mar_lin,im_y-mar_lin),(0,0,255),4)

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
				#cv2.rectangle(frame,(x,y),(x+w,y+h),(0,255,0),2)


				X_des = x+(w/2)
				Y_des = y+(h/2)
				cv2.rectangle(frame,(X_des-(Track_tol/2)-20,Y_des-(Track_tol/2)-20),(X_des+(Track_tol/2)+20,Y_des+(Track_tol/2)+20),(0,255,0),2)
				det_r = 4
				cv2.circle(frame,(X_des,Y_des), det_r, (0,255,0), -1)

				X_er = (X_des-im_x)
				Y_er = (Y_des-im_y)

				if dt == 0:
					Vx = 0
					Vy = 0
				else:
					Vx = (X_des-x_pre)/dt
					Vy = (Y_des-y_pre)/dt


				if abs(X_er) < Track_tol:
					P = 0
				else:
					P = -kpx*X_er-(kdx*Vx)

				if abs(Y_er) < Track_tol:
					T = 0
				else:
					T = -kpy*Y_er-(kdy*Vy)
							# Differential Controller Code


				print("Desired Corrections Pan: "+repr(P)+" and  Tilt: "+repr(T)+"\n")
				# rospy.sleep(0.1)
				self.move_arm(P,T)
				x_pre = X_des
				y_pre = Y_des
			# # Did not detect any contours, then sweep
			else:
				homing_count +=1
				if homing_count >= max_homing_count:
					self.set_homing_position(initial = False)
					homing_count = 0
				# self.pan_sweep()
				# self.tilt_sweep()

			# show the frame to our screen
			cv2.putText(mask,'Processed Feed',(width-240,height-5), font, 0.9,(255,255,255),2)
			cv2.imshow("Frame", frame)
			cv2.imshow("Tresholded", mask)
			key = cv2.waitKey(1) & 0xFF
			dt = time.time()-t
			#print("Loop Execution Time: "+repr(dt)+" seconds"+"\n")
			# if the 'q' key is pressed, stop the loop
			if key == ord("q"):
				break
		# rospy.spin()


if __name__=='__main__':
	try:
		arm = ArmMovement()
		arm.image_processing()
		# arm.subscribe()
		# rospy.spin()
		# while True:
		# 	rospy.sleep(0.5)
		# 	arm.pan_sweep()
		# 	arm.tilt_sweep()
		# 	if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
		# 		line = raw_input()
		# 		break
		# arm.__test_joint_movement__()
		#arm.__test_echo_current_joint_values__()
	except rospy.ROSInterruptException:
		pass
