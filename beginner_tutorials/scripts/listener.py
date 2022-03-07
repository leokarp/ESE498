#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.data)

def listener():

    
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber('accel_topic', Float32, callback)
    
    print('accel_node')

    #spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
