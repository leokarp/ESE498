#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32MultiArray
import RPi.GPIO as GPIO
import sys
import time
import argparse
#import busio
import smbus

from picamera import PiCamera
import cv2 as cv
import numpy as np
import os

global cX
global cY

cap = cv.VideoCapture(0)

def talker():
    pub = rospy.Publisher('camera_topic', Float32MultiArray, queue_size = 1)
    rospy.init_node('camera_node', anonymous  = True)
    rate = rospy.Rate(10)
    
    # Take each frame
    _, frame = cap.read()
    # Convert BGR to HSV
    hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
    # define range of purple color in HSV
    lower_blue = np.array([120,100,0])
    upper_blue = np.array([160,255,255])
    # Threshold the HSV image to get only blue colors
    mask = cv.inRange(hsv, lower_blue, upper_blue)
    # Bitwise-AND mask and original image
    res = cv.bitwise_and(frame,frame, mask= mask)
#     cv.imshow('frame',frame)
#     cv.imshow('mask',mask)
#     cv.imshow('res',res)
    
    M = cv.moments(mask)
    
    if int(M["m00"]) == 0:
        cX=999
        cY=999
    else:
        cX = int(M["m10"] / M["m00"])          
        cY = int(M["m01"] / M["m00"])
        
    #print (f"Center: ({cX} , {cY})")
    
    cv.destroyAllWindows()
    
   #------------------------------
    c = [cX, cY]
    print(c)
    
    my_msg = Float32MultiArray()
    my_msg.data = c
    pub.publish(my_msg)
    
    rate.sleep()

if __name__ == '__main__':
   while True:
      try:
         talker()
      except rospy.ROSInterruptException:
         break
