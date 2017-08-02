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

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python', anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group = moveit_commander.MoveGroupCommander("arm")
# joint_defaults = getJointsFromURDF()
# joints = rospy.get_param('/arbotix/joints', dict())
# joint_limits = list()
# # Get joint limits
# for name in sorted(joints.keys()):
#   # pull angles
#   min_angle, max_angle = getJointLimits(name, joint_defaults)
#   joint_limits.append({'name':name, 'min_angle':min_angle*math.pi/180.0, 'max_angle':max_angle*math.pi/180.0})
# max_limits = [joint_limits[i].get('max_angle')*180.0/math.pi*100 for i in range(0,5)]
# min_limits = [joint_limits[i].get('min_angle')*180.0/math.pi*100 for i in range(0,5)]
pan_index  = 0
tilt_index = 3
pan_angle_min  = -90.0*math.pi/180.0
pan_angle_max  =  90.0*math.pi/180.0
tilt_angle_min = -90.0*math.pi/180.0
tilt_angle_max =  90.0*math.pi/180.0

def move_joints(data):
  [pan,tilt] = data.data
  # print "== Pan and Tilt ",(pan,tilt)
  group_variable_values = group.get_current_joint_values()
  # print "============ Joint values: ", group_variable_values

  ## Then, we will get the current set of joint values for the group

  ## Now, let's modify one of the joints, plan to the new joint
  ## space goal and visualize the plan

  # if   group_variable_values[pan_index]+pan > pan_angle_max:
  #   group_variable_values[pan_index] = pan_angle_max
  # elif group_variable_values[pan_index]+pan < pan_angle_min:
  #   group_variable_values[pan_index] = pan_angle_min
  # else:
  #   group_variable_values[pan_index] += pan
  
  # if   group_variable_values[tilt_index]+tilt > tilt_angle_max:
  #   group_variable_values[tilt_index] = tilt_angle_max
  # elif group_variable_values[tilt_index]+tilt < tilt_angle_min:
  #   group_variable_values[tilt_index] = tilt_angle_min
  # else:
  #   group_variable_values[tilt_index] += tilt

  group_variable_values[pan_index] += pan
  group_variable_values[tilt_index] += tilt
  try:
    group.set_joint_value_target(group_variable_values)
    # plan2 = group.plan()
    group.go(wait=True)
  except:
    pass

def move_group_python_interface_tutorial():


  display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory,queue_size=10)
  rospy.Subscriber('panTilt',std_msgs.msg.Float64MultiArray,move_joints)

  # ## We can get the name of the reference frame for this robot
  # print "============ Reference frame: %s" % group.get_planning_frame()
  # ## We can also print the name of the end-effector link for this group
  # print "============ Reference frame: %s" % group.get_end_effector_link()
  # ## We can get a list of all the groups in the robot
  # print "============ Robot Groups:"
  # print robot.get_group_names()
  # ## Sometimes for debugging it is useful to print the entire state of the
  # ## robot.
  # print "============ Printing robot state"
  # print robot.get_current_state()
  # print "============"

  # position_list = list()
  # orientation_list = list()
  
  # pose_list = list()
  # file = open('pose_list.txt','w')
  
  
  # # print "Pos_init \n %s" %position
  # # position.y += 0.000001
  # # print "Pos in between y: %s" %position.y
  # # plan = group.plan()
  # # group.go(wait=True)
  # # rospy.sleep(2)
  # # print "Pos_final \n %s" %position
  # # print "============ Entering loop"
  # # while plan:
  # #   position.y += 0.001
  # #   plan = group.plan()
  # #   if plan != True:
  # #    plan = False
  # #   pose_list.append({'position' : position, 'orientation' : orientation, 'possible' : plan})
  # #   group.go(wait=True)
  # #   if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
  # #     line = raw_input()
  # #     break
  # t = PrettyTable(['position.x', 'position.y','position.z','orientation.x','orientation.y','orientation.z','orientation.w'])
  # print "============ Entering loop"
  # while True:
  #   current_pose = group.get_current_pose()
  #   position = current_pose.pose.position
  #   orientation = current_pose.pose.orientation
  #   t.add_row([position.x,position.y,position.z,orientation.x,orientation.y,orientation.z,orientation.w])
  #   print t
  #   if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
  #     line = raw_input()
  #     pose_list.append({'position': position, 'orientation' : orientation})
  #     if line == 'q':
  #       break
  #     continue

  # file.write(str(pose_list))
  # file.close()

  # print "============ Generating plan 1"
  # current_pose = group.get_current_pose()
  # pose_target = current_pose.pose
  # print "Current pose: %s" %current_pose.pose
  # pose_target.orientation.w = 0.0
  # pose_target.orientation.x = 0.0
  # pose_target.orientation.y = 0.0
  # pose_target.orientation.z = 1.0
  # # pose_target.position.x = 0.65
  # # pose_target.position.y = -1.0
  # # pose_target.position.z = 1.0
  # group.set_pose_target(pose_target)
  # #group.set_random_target()
  # group.go(wait=True)

  # plan1 = group.plan()
  # rospy.sleep(5)
  # print "============ Visualizing plan1"
  # display_trajectory = moveit_msgs.msg.DisplayTrajectory()

  # display_trajectory.trajectory_start = robot.get_current_state()
  # display_trajectory.trajectory.append(plan1)
  # display_trajectory_publisher.publish(display_trajectory);

  # print "============ Waiting while plan1 is visualized before execution on robot..."
  # rospy.sleep(5)

  # while True:
  #   group_variable_values = group.get_current_joint_values()
  #   print "Max Angles", [joint_limits[i].get('max_angle')*180.0/math.pi*100 for i in range(0,5)]
  #   print "Min Angles", [joint_limits[i].get('min_angle')*180.0/math.pi*100 for i in range(0,5)]
  #   print "Current joint angles:", [ele*180.0/math.pi for ele in group_variable_values]
  #   try:
  #     angle = float(input("Enter angle to turn: "))
  #     index = int(input("Enter index: "))
  #     print "=========== Moving by ", angle, " degrees"
  #     group_variable_values[index] += angle*math.pi/180.0
  #     print "New angles will be:", [ele*180.0/math.pi for ele in group_variable_values]
  #     group.set_joint_value_target(group_variable_values)
  #     group.go(group_variable_values, wait=True)
  #   except:
  #     print "Exiting!"
  #     break
  # group_variable_values = group.get_current_joint_values()
  # angle = 90.0
  # group_variable_values[0] = angle*math.pi/180.0/100
  # group.set_joint_value_target(group_variable_values)
  # group.go(wait=True)
  # t = PrettyTable(['0','1','2','3','4'])
  # while True:
  #   group_variable_values = group.get_current_joint_values()
  #   t.add_row([ele*180.0/math.pi for ele in group_variable_values])
  #   print t
  #   # print "Current joint angles:", [ele*180.0/math.pi*100 for ele in group_variable_values]
  #   if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
  #     line = raw_input()
  #     break

  # print "============ The current pose of the robot is %s" % position


  # group.clear_pose_targets()

  rospy.spin()
  ## When finished shut down moveit_commander.
  moveit_commander.roscpp_shutdown()
  rospy.sleep(1)
  print "============ STOPPING"

def loop_exit(condition,execute):
  while condition:
    execute
    if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
      line = raw_input()
      break

def loop_continue(condition,execute):
  while condition:
    execute
    if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
      line = raw_input()
      continue

if __name__=='__main__':
  try:
    move_group_python_interface_tutorial()
  except rospy.ROSInterruptException:
    pass
