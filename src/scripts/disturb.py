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

class Disturbance():
	def __init__(self):
		moveit_commander.roscpp_initialize(sys.argv)
		rospy.init_node('move_group_python', anonymous=True)
		self.group = moveit_commander.MoveGroupCommander("arm")
		self.disturbance_index = 1
		self.disturbance_down = False
		self.disturbance_value = math.pi/180.0*10.0

	def __del__(self):
		## When finished shut down moveit_commander.
		moveit_commander.roscpp_shutdown()

	def set_disturbance_index(self,disturbance_index):
		self.disturbance_index = disturbance_index

	def get_disturbance_index(self):
		return self.disturbance_index

	def set_disturbance_value(self,disturbance_value):
		self.disturbance_value = disturbance_value

	def get_disturbance_value(self):
		return self.disturbance_value

	def introduce_disturbance(self,disturbance_value):
		group_variable_values = self.group.get_current_joint_values()
		group_variable_values_copy = self.group.get_current_joint_values()
		group_variable_values[self.disturbance_index] += disturbance_value
		try:
			self.group.set_joint_value_target(group_variable_values)
			# plan2 = group.plan()
			self.group.go(wait=True)
		except:
			self.group.set_joint_value_target(group_variable_values_copy)
			# plan2 = group.plan()
			self.group.go(wait=True)
			print "No plan found. Setting to original value"

	def disturb(self):
		while True:
			if self.disturbance_down:
				self.introduce_disturbance(self.disturbance_value)
				self.disturbance_down = False
			else:
				self.introduce_disturbance(-self.disturbance_value/2)
				self.disturbance_down = True
	def set_new_home_position(self):
		

if __name__=='__main__':
	try:
		arm = Disturbance()
		arm.disturb()
	except rospy.ROSInterruptException:
		pass