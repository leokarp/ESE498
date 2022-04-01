import busio
import math
import time
#import rospy
import signal
from board import SCL, SDA
#from std_msgs.msg import Int16
import time
from adafruit_pca9685 import PCA9685

#signal.signal(signal.SIGINT, signal.SIG_IGN)
# KEY: Forward = 20%, Backwards = 10%, Stop/Neutral = 15%

# 16-bit duty cycle has max value 2^16 - 1 = 65535
DUTY_10 = 6553
DUTY_15 = 9830
DUTY_20 = 13107
RANGE = 3277

INIT_SLEEP_TIME = 3
BACK_SLEEP_TIME = .1
GEN_SLEEP_TIME = 1


i2c = busio.I2C(SCL, SDA)
time.sleep(1)
pca = PCA9685(i2c)
time.sleep(1)
pca.frequency = 100
channel = pca.channels[7]


def init_motor():
   channel.duty_cycle = DUTY_15
   cont_permission = input(
   "Hold down EZ Set until the LED blinks RED.\n"
   "Then press any key to continue.\n")
   print("starting")
   time.sleep(INIT_SLEEP_TIME)
   channel.duty_cycle = DUTY_20
   time.sleep(INIT_SLEEP_TIME)
   channel.duty_cycle = DUTY_15
   time.sleep(INIT_SLEEP_TIME)
   channel.duty_cycle = DUTY_10
   time.sleep(INIT_SLEEP_TIME)
   channel.duty_cycle = DUTY_15
   time.sleep(INIT_SLEEP_TIME)
   print("done")


def before_backwards():
   channel.duty_cycle = DUTY_15
   time.sleep(BACK_SLEEP_TIME)
   channel.duty_cycle = DUTY_10
   time.sleep(BACK_SLEEP_TIME)
   channel.duty_cycle = DUTY_15
   time.sleep(BACK_SLEEP_TIME)


def percent_to_duty_cycle(percent):
   if (0 < percent) and (percent < 15):
      percent = 15
   elif (percent > 100):
      percent = 100
   elif (0 > percent) and (percent > -15):
      percent = -15
   elif (percent < -100):
      percent = -100
   true_percent = percent / 100
   increment = true_percent * RANGE
   channel.duty_cycle = DUTY_15 + math.floor(increment)


def callback(data):
   caller_id = rospy.get_caller_id()
   log_string = f"ID: {caller_id} SPEED: {data.data} %"
   rospy.loginfo(log_string)
   if data.data < 0:
      before_backwards()
   percent_to_duty_cycle(data.data)

def receiver():
   rospy.init_node('motor', anonymous=True)
   rospy.Subscriber('speed', Int16, callback)
   rospy.spin()

if __name__ == '__main__':
   init_motor()
   #receiver()