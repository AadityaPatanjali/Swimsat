#!/usr/bin/env python

import sys
import select
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import std_msgs.msg
from prettytable import PrettyTable
from arbotix_python.joints import *
import math
import cv2

class ArmMovement():
	def __init__(self):
		moveit_commander.roscpp_initialize(sys.argv)
		rospy.init_node('move_group_python', anonymous=True)
		self.robot = moveit_commander.RobotCommander()
		self.scene = moveit_commander.PlanningSceneInterface()
		self.group = moveit_commander.MoveGroupCommander("arm")
		self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory,queue_size=10)
		# rospy.Subscriber('panTilt',std_msgs.msg.Float64MultiArray,self.move_joints_callback)
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
		self.set_homing_position()		
		## When finished shut down moveit_commander.
		moveit_commander.roscpp_shutdown()

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

	def set_homing_position(self):
		angles = [0.0,52.0,-80.0,-60.0,0.0]
		group_variable_values = [ele*math.pi/180.0 for ele in angles]
		self.group.set_joint_value_target(group_variable_values)
		self.group.go(wait=True)

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
		group_variable_values[self.pan_index] += pan
		group_variable_values[self.tilt_index] += tilt
		try:
			self.group.set_joint_value_target(group_variable_values)
			# plan2 = group.plan()
			self.group.go(wait=True)
		except:
			pass

	def move_arm(self,pan,tilt):
		group_variable_values = self.group.get_current_joint_values()
		group_variable_values_copy = self.group.get_current_joint_values()
		group_variable_values[self.pan_index] += pan
		group_variable_values[self.tilt_index] += tilt
		try:
			self.group.set_joint_value_target(group_variable_values)
			# plan2 = group.plan()
			self.group.go(wait=False)
		except:
			self.group.set_joint_value_target(group_variable_values_copy)
			# plan2 = group.plan()
			self.group.go(wait=False)

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
		t = PrettyTable(['0','1','2','3','4'])
		while True:
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

	def main(self):
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
		
		camera = cv2.VideoCapture(1)

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
				# rospy.sleep(0.1)
				self.move_arm(P,T)
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
		arm = ArmMovement()
		arm.main()
		# while True:
		# 	rospy.sleep(0.5)
		# 	arm.pan_sweep()
		# 	arm.tilt_sweep()
		# 	if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
		# 		line = raw_input()
		# 		break
	    # arm.__test_echo_current_joint_values__()
	    # arm.__test_joint_movement__()
	except rospy.ROSInterruptException:
		pass