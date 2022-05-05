#BGR to HSV converter:

import cv2 as cv
import numpy as np

#fill these in:
R = 68
G = 48
B = 44

#calculate:
color = np.uint8([[[B,G,R]]])
hsv_color = cv.cvtColor(color,cv.COLOR_BGR2HSV)

#print result
print(hsv_color)