#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32MultiArray
import RPi.GPIO as GPIO
import sys
import time
import argparse
import busio
import smbus
# Simple example of reading the MCP3008 analog input channels and printing
# them all out.
# Author: Tony DiCola
# License: Public Domain

# Import SPI library (for hardware SPI) and MCP3008 library.
import Adafruit_GPIO.SPI as SPI
import Adafruit_MCP3008

def talker():
   #Needed for publisher
    pub = rospy.Publisher('linetracker_topic', Float32MultiArray, queue_size = 1)
    rospy.init_node('linetracker_node', anonymous  = True)
    rate = rospy.Rate(5)

    # Software SPI configuration:
    # CLK  = 18
    # MISO = 23
    # MOSI = 24
    # CS   = 25
    # mcp = Adafruit_MCP3008.MCP3008(clk=CLK, cs=CS, miso=MISO, mosi=MOSI)

    # Hardware SPI configuration:
    SPI_PORT   = 0
    SPI_DEVICE = 0
    mcp = Adafruit_MCP3008.MCP3008(spi=SPI.SpiDev(SPI_PORT, SPI_DEVICE))


    print('Reading MCP3008 values, press Ctrl-C to quit...')
    # Print nice channel column headers.
    print('| {0:>4} | {1:>4} | {2:>4} | {3:>4} | {4:>4} |'.format(*range(8)))
    print('-' * 57)
    # Main program loop.
    while True:
        # Read all the ADC channel values in a list.
        values = [0]*5
        for i in range(5):
            # The read_adc function will get the value of the specified channel (0-7).
            values[i] = mcp.read_adc(i)
        # Print the ADC values.
        print('| {0:>4} | {1:>4} | {2:>4} | {3:>4} | {4:>4} |'.format(*values))
        # Pause for half a second.
    
    pub.publish(values[i])

    rate.sleep()
    
    GPIO.cleanup()


if __name__ == '__main__':
   while True:
      try:
         talker()
      except rospy.ROSInterruptException:
         break