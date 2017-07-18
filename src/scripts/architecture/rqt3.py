#!/usr/bin/env python
import rospy
from std_msgs.msg import String

def callback(data):
		return

def talker():
    rospy.init_node('Webcam_Node', anonymous=True)
    pub = rospy.Publisher('ext_img', String, queue_size=10)



    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
