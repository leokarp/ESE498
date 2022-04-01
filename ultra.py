#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32
import RPi.GPIO as GPIO
import sys
import time
import argparse
import busio
import smbus

def talker():
   #Needed for publisher
    pub = rospy.Publisher('ultra_topic', Float32, queue_size = 1)
    rospy.init_node('ultra_node', anonymous  = True)
    rate = rospy.Rate(5)

   #Real code specific to node
    GPIO.setmode(GPIO.BOARD)
    PIN_TRIGGER = 7
    PIN_ECHO = 11
       
    GPIO.setup(PIN_TRIGGER, GPIO.OUT)
    GPIO.setup(PIN_ECHO, GPIO.IN)
    GPIO.output(PIN_TRIGGER, GPIO.LOW)
    GPIO.output(PIN_TRIGGER, GPIO.HIGH)
              
    GPIO.output(PIN_TRIGGER, GPIO.LOW)
    
    
    while GPIO.input(PIN_ECHO)==0:
        pulse_start_time = time.time()
    while GPIO.input(PIN_ECHO)==1:
       pulse_end_time = time.time()
           
    pulse_duration = pulse_end_time - pulse_start_time

    distance = round(pulse_duration * 17150, 2)

    print ("Distance:",distance,"cm")

    pub.publish(distance)

    rate.sleep()
    
    GPIO.cleanup()


if __name__ == '__main__':
   while True:
      try:
         talker()
      except rospy.ROSInterruptException:
         break