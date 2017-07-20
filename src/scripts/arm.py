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




	def move_joints_callback(self,data):
		[pan,tilt] = data.data
		pan_index  = 0
		tilt_index = 3
		# print "== Pan and Tilt ",(pan,tilt)
		group_variable_values = self.group.get_current_joint_values()
		group_variable_values[pan_index] += pan
		group_variable_values[tilt_index] += tilt
		try:
			self.group.set_joint_value_target(group_variable_values)
			# plan2 = group.plan()
			self.group.go(wait=True)
		except:
			pass

	def move_arm(self,pan,tilt,pan_index=0,tilt_index=3):
		group_variable_values = self.group.get_current_joint_values()
		group_variable_values[pan_index] += pan
		group_variable_values[tilt_index] += tilt
		try:
			self.group.set_joint_value_target(group_variable_values)
			# plan2 = group.plan()
			self.group.go(wait=False)
		except:
			pass

	def get_current_joint_values(self):
		return self.group.get_current_joint_values()


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
			print "Max Angles", [self.joint_limits[i].get('max_angle')*180.0/math.pi*100 for i in range(0,5)]
			print "Min Angles", [self.joint_limits[i].get('min_angle')*180.0/math.pi*100 for i in range(0,5)]
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

	def __test_single_joint_movement__(self,index,angle):
		group_variable_values = self.group.get_current_joint_values()
		group_variable_values[index] = angle*math.pi/180.0/100
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
		Track_tol = 50
		X_Step = 4*math.pi/180
		Y_Step = 4*math.pi/180
		P = 0
		T = 0
		# HSV Limits
		greenLower = (0, 119, 0)
		greenUpper = (5, 255, 255)
		
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

				elif X_er > 0:
					P = -X_Step

				elif X_er < 0:
					P =  X_Step

				if abs(Y_er) < Track_tol:
					T = 0

				elif Y_er > 0:
					T = -Y_Step

				elif Y_er < 0:
					T = Y_Step
				print("Desired Corrections Pan: "+repr(P)+" and  Tilt: "+repr(T)+"\n")
				# rospy.sleep(0.1)
				self.move_arm(P,T)
			# show the frame to our screen
			cv2.imshow("Frame", frame)
			
			key = cv2.waitKey(1) & 0xFF
			# if the 'q' key is pressed, stop the loop
			if key == ord("q"):
				break
		# rospy.spin()
		## When finished shut down moveit_commander.
		moveit_commander.roscpp_shutdown()


if __name__=='__main__':
  try:
    arm = ArmMovement()
    arm.main()
    # arm.__test_joint_movement__()
  except rospy.ROSInterruptException:
    pass