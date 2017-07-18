#!/usr/bin/env python

import rospy
import wx

from math import radians

from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import *
from arbotix_msgs.srv import Relax
from arbotix_python.joints import *

class Commander():

    def __init__(self):
        rospy.init_node('Camera')
        self.desired_pos = rospy.Publisher('desired_pos', JointState, queue_size=5)
        self.servo_pos = rospy.Publisher('servo_pos', String, queue_size=5)
        rospy.loginfo('servo_states=%s','servo_pos')
        rospy.Subscriber('joint_states', JointState, self.stateCb)

        self.servos = list()
        self.publishers = list()
        self.relaxers = list()
        joint_defaults = getJointsFromURDF()
        i=0
        joints = rospy.get_param('/arbotix/joints', dict())
        # create sliders and publishers
        for name in sorted(joints.keys()):
            # pull angles
            min_angle, max_angle = getJointLimits(name, joint_defaults)
            # create slider
            s = {'index':i, 'name':name, 'min_angle':min_angle, 'max_angle':max_angle, 'position':0}
            self.servos.append(s)
            i += 1



    def stateCb(self,msg):
        self.desired_pos.publish(msg)
        for servo in self.servos:
            try:
                idx = msg.name.index(servo.name)
                servo.position = msg.position[idx]
            except: 
                pass
        rospy.loginfo(String(self.servos))
        self.servo_pos.publish(str(self.servos))

    def main(self):
        
        self.rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
          self.rate.sleep()

        rospy.spin()


        

if __name__ == '__main__':
    try:
        command = Commander()
        command.main()
    except rospy.ROSInterruptException:
        pass


