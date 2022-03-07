#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT
import rospy
from std_msgs.msg import Float32
import RPi.GPIO as GPIO
import sys
import time
import argparse
#import busio
import smbus

import time
import board
import adafruit_mpu6050

i2c = board.I2C()  # uses board.SCL and board.SDA
mpu = adafruit_mpu6050.MPU6050(i2c)

def talker():
   #Needed for publisher
   pub = rospy.Publisher('accel_topic', Float32, queue_size = 1)
   rospy.init_node('accel_node', anonymous  = True)
   rate = rospy.Rate(10)

   #Real code specific to node
  
   #print(x)
   
   print("Acceleration: X:%.2f, Y: %.2f, Z: %.2f m/s^2" % (mpu.acceleration))
   #print("Gyro X:%.2f, Y: %.2f, Z: %.2f rad/s" % (mpu.gyro))
   #print("Temperature: %.2f C" % mpu.temperature)
   print(mpu.acceleration[1])
   print(type(mpu.acceleration))
   pub.publish(mpu.acceleration[1])
   
   print("")
   
   rate.sleep()

if __name__ == '__main__':
   while True:
      try:
         talker()
      except rospy.ROSInterruptException:
         break