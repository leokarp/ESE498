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
import keyboard


# This code is adapted from code from a previous semester's pi car project by Alejandro Acevedo Guillot, Ishan Gauli, and William Zachary Vernon
# https://github.com/alejandroacevedoguillot/Capstone-PiCar/blob/master/Python_Scripts/Motor_Steering.py


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
    
    

#Initialize Listener Variables
#global distance
distance = None
cX = None
cY = None


def callbackUltra(data):
    rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.data)
    global distance
    distance = data.data
       
    
def callbackCamera(data):
    rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.data)
    global cX
    global cY
    c = data.data
    cX = c[0]
    cY = c[1]


def listener():

    
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber('ultra_topic', Float32, callbackUltra)
    
    
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber('camera_topic', Float32MultiArray, callbackCamera)

    count = 0
    tic = time.time()
    cnt = 0
    while True:
        #print(distance)
        #print([cX, cY])
        time.sleep(1/10)
        count = count + 1

        spacing = 250
        toc =  time.time() - tic
        if cX is not None and distance is not None and cY is not None:
            if cY < 120:
                Steering(pca,155)
                time.sleep(0.7) #2
                Steering(pca,90)
                time.sleep(1)

            else:
                if cX < 150: #150
                     Steering(pca,92.5) #95
                     if cY > 160:
                         Motor_Speed(pca,0.2)
                     else:
                         Motor_Speed(pca,0.15)
                elif cX > 170: #170
                    Steering(pca, 87.5) #85
                    if cY > 160:
                         Motor_Speed(pca,0.2)
                    else:
                         Motor_Speed(pca,0.15)
                else:
                    Steering(pca,90)
                    if cY > 160:
                         Motor_Speed(pca,0.2)
                    else:
                         Motor_Speed(pca,0.15)
        
        if count==1250: 
            print('stopped')
            break  # finishing the loop
                    
 

    Motor_Speed(pca,0)
    Steering(pca,90)
            

    #spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
    


if __name__ == '__main__':
    pca=Servo_Motor_Initialization()
    Motor_Speed(pca,0) #0.15
    Steering(pca,90)
    time.sleep(1)
   
    listener()

        
