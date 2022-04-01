#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray


#CODE FROM PWM.py
from board import SCL,SDA
import busio
from adafruit_pca9685 import PCA9685
import time
import adafruit_motor.servo
import math

def Servo_Motor_Initialization():
    # Create busio i2C bus instance to communicate with driver.
    i2c_bus = busio.I2C(SCL,SDA)
    # Start Communicating with Driver
    pca =PCA9685(i2c_bus)
    # Set Frequency
    pca.frequency =  100
    #kit = ServoKit(channels=16)
   
    return pca

def Motor_Start(pca):
    # This function set the servo broad to min pwm to run.
    # Motor is always set to channel 7 of the broad.
    pca.channels[7].duty_cycle = 9830
    
def Motor_Speed(pca,percent):
    # Converts a -1 to 1 value to 16-bit duty cycle
    # 20% full forward
    # 15% corresponds to zero speed
    # 10% full reverse
    speed = (percent*3276) + 65535 * 0.15
    pca.channels[7].duty_cycle = math.floor(speed)

     
def Steering(pca,angle):
    # Limiting
    if angle >180:
        angle = 180
    if angle < 0:
        angle = 0
    # Converts to 16-bit duty between 10% and 20%    
    duty = ((angle/180)*6553)+6553
    pca.channels[1].duty_cycle= math.floor(duty)
    
def Pan(pca,angle):
    #Limits and center range
    offset =10
    if angle > 180:
        angle =180
    if angle < 20:
        angle =20
    # Apply offset and convert
    duty = (((angle+offset)/180)*6553)+6553
    pca.channels[4].duty_cycle= math.floor(duty)
       
def Tilt(pca,angle):
     #Limits and center range
    offset = 20
    if angle > 170:
        angle =170
    if angle < 30:
        angle =30
    # Apply offset and convert    
    duty = (((angle+offset)/180)*6553)+6553
    pca.channels[5].duty_cycle= math.floor(duty)
    
def Motor_Speed2(pca,int_value):
    pca.channels[7].duty_cycle = int_value

#Initialize Listener Variables
#global distance
distance = None
yAccel = None
xAccel = None
zAccel = None
lineTrack = None

# myvar = None
# counter = 0

# def manipulator(reading, sensor):
#     print('scoobbby',reading)
#     print(type(rospy.get_caller_id()))
#     print(sensor)
#     if counter % 2 == 0:
#         print('ultrasonic', reading)
#     else:
#         print('accelerometer',reading)
        
# def callback(data):
#     rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.data)
#     global myvar
#     global counter
#     myvar = data
#     counter = counter + 1
#     manipulator(myvar, counter)

def callbackUltra(data):
    rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.data)
    global distance
    distance = data.data
    #print(distance)
    
    
    
def callbackAccel(data):
    rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.data)
    global xAccel
    global yAccel
    global zAccel
    xAccel = data.data
    
def callbackTracker(data):
    rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.data)
    global lineTrack
    print(data.data)
    lineTrack = data.data


def listener():

    
#     rospy.init_node('listener', anonymous=True)
# 
#     rospy.Subscriber('accel_topic', Float32, callbackAccel)
    
    #rospy.init_node('listener', anonymous=True)

    #rospy.Subscriber('ultra_topic', Float32, callbackUltra)
    
    
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber('linetracker_topic', Float32MultiArray, callbackTracker)
    
    #print(distance)
    count = 0
    while True:
        #print(distance)
        #print(lineTrack)
        time.sleep(1/5)
        count = count + 1
        
        
        if distance is not None:
            
            if distance < 7:
                Motor_Speed(pca,0)
        
        if count == 50:
            break
#     rospy.init_node('listener', anonymous=True)
# 
#     rospy.Subscriber('tracker_topic', Float32, callbackTracker)

    #spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
    


if __name__ == '__main__':
    pca=Servo_Motor_Initialization()
#     print("Motor Initialized")
#     
#     m = 1
# 
#     pca.channels[m].duty_cycle = 13107
#     time.sleep(3)
#     pca.channels[m].duty_cycle = 9830
#     time.sleep(3)
#     pca.channels[m].duty_cycle = 6553
#     time.sleep(3)
#     pca.channels[m].duty_cycle = 9830
# 
#     Motor_Start(pca)
#     print("Motor Started")
#     time.sleep(3)
# 
#     Motor_Speed(pca,0.2)
#     print("Moving Forward")
#     time.sleep(3)
# 
#     Motor_Speed(pca,0.15)
#     print("Stopped")
#     time.sleep(3)
# 
#     Motor_Speed(pca,0.1)
#     print("Moving Backwards")
#     time.sleep(3)
# 
#     Motor_Speed(pca,0.15)
#     print("Stopped")
#     time.sleep(3)

    #Motor_Speed(pca,0.2)

    listener()
