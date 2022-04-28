from picamera import PiCamera
import os
import time
import cv2 as cv
import numpy as np

global cX
global cY

PiCamera.resolution = (640, 480)

PiCamera.shutter_speed = 100

PiCamera.crop = (100, 100, 100, 100)

cap = cv.VideoCapture(0)

while(1):
    # Take each frame
    _, frame = cap.read()
    # Convert BGR to HSV
    hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
    # define range of blue color in HSV
    lower_blue = np.array([120,100,0]) #120
    upper_blue = np.array([160,255,255]) #160
    # Threshold the HSV image to get only blue colors
    mask = cv.inRange(hsv, lower_blue, upper_blue)
    # Bitwise-AND mask and original image
    res = cv.bitwise_and(frame,frame, mask= mask)
    cv.imshow('frame',frame)
    cv.imshow('mask',mask)
    cv.imshow('res',res)
    k = cv.waitKey(5) & 0xFF
    if k == 27:
        break
    
    M = cv.moments(mask)
    
    if int(M["m00"]) == 0:
        cX=999
        cY=999
    else:
        cX = int(M["m10"] / M["m00"])          
        cY = int(M["m01"] / M["m00"])
        
    print (f"Center: ({cX} , {cY})")
    
cv.destroyAllWindows()
