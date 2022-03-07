#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32
import RPi.GPIO as GPIO
import sys
import time
import argparse
#import busio
import smbus
from picamera import PiCamera

def talker():
   #Needed for publisher
   pub = rospy.Publisher('test_topic', Float32, queue_size = 1)
   rospy.init_node('test_node', anonymous  = True)
   rate = rospy.Rate(10)

   #Real code specific to node
   camera = PiCamera()
   camera.capture('testing1.png')
   camera.resolution = (320,208)
   camera.capture('testing2.png')

   pub.publish("something")
   rate.sleep()

if __name__ == '__main__':
   while True:
      try:
         talker()
      except rospy.ROSInterruptException:
         break