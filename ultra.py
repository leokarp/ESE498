#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32
import RPi.GPIO as GPIO
import sys
import time
import argparse
import busio
import smbus

global distArray
distArray = [1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000]
def talker():
   #Needed for publisher
    counter = 0
    while True:
        pub = rospy.Publisher('ultra_topic', Float32, queue_size = 1)
        rospy.init_node('ultra_node', anonymous  = True)
        rate = rospy.Rate(10)

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
    
    
#     GPIO.cleanup()
#     
#     GPIO.setmode(GPIO.BOARD)
#     PIN_TRIGGER = 7
#     PIN_ECHO = 11
#        
#     GPIO.setup(PIN_TRIGGER, GPIO.OUT)
#     GPIO.setup(PIN_ECHO, GPIO.IN)
#     GPIO.output(PIN_TRIGGER, GPIO.LOW)
#     GPIO.output(PIN_TRIGGER, GPIO.HIGH)
#               
#     GPIO.output(PIN_TRIGGER, GPIO.LOW)
#         #print ("Distance:",distance,"cm")
#     
#     while GPIO.input(PIN_ECHO)==0:
#         pulse_start_time = time.time()
#     while GPIO.input(PIN_ECHO)==1:
#         pulse_end_time = time.time()
#            
#     pulse_duration = pulse_end_time - pulse_start_time
# 
#     distance2 = round(pulse_duration * 17150, 2)
#     
#     GPIO.cleanup()
# 
#     GPIO.setmode(GPIO.BOARD)
#     PIN_TRIGGER = 7
#     PIN_ECHO = 11
#        
#     GPIO.setup(PIN_TRIGGER, GPIO.OUT)
#     GPIO.setup(PIN_ECHO, GPIO.IN)
#     GPIO.output(PIN_TRIGGER, GPIO.LOW)
#     GPIO.output(PIN_TRIGGER, GPIO.HIGH)
#               
#     GPIO.output(PIN_TRIGGER, GPIO.LOW)
# 
#     while GPIO.input(PIN_ECHO)==0:
#         pulse_start_time = time.time()
#     while GPIO.input(PIN_ECHO)==1:
#         pulse_end_time = time.time()
#            
#     pulse_duration = pulse_end_time - pulse_start_time
# 
#     distance3 = round(pulse_duration * 17150, 2)
#     
#     distance = (distance1+distance2+distance3)/3

        distArray[counter] = distance
        #print(counter)
        counter = counter + 1
    
        if counter == 10:
            counter = 0
            
    
        true_distance = (distArray[0]+distArray[1]+distArray[2]+distArray[3]+distArray[4]+distArray[5]+distArray[6]+distArray[7]+distArray[8]+distArray[9])/10
        print(true_distance)
        pub.publish(true_distance)

        rate.sleep()
    
        GPIO.cleanup()


if __name__ == '__main__':
   while True:
      try:
          talker()
      except rospy.ROSInterruptException:
         break