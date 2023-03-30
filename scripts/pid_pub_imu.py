#!/usr/bin/env python

import rospy
import math
from std_msgs.msg import Int32
from geometry_msgs.msg import Vector3
from simple_pid import PID
from std_msgs.msg import Bool

set_point=0
current_value=0
pwm_voltage=0
accelx=0
accely=0
accelz=0
pid= PID(0,0,0,0)

def callback(data):
    global accelx,accely,accelz
    accelx=data.x
    accely=data.y
    accelz=data.z
    ori_calc()

def callback2(data1):
    global set_point,pid
    set_point=data1.data
    pid = PID(0.5, 0.01, 0.1, setpoint=set_point)

def shut_down(data2):
    rospy.loginfo(f"Shutdown!!!!")
    if(data2.data==0):
        rospy.signal_shutdown("Button pressed")  

def ori_calc():
    global accelx,accely,accelz,current_value
    current_value=(math.atan2(-accelx,math.sqrt((accely**2)+(accelz**2))))*180/3.14      


def talker():
    global current_value,set_point,pwm_voltage,pid
    pub = rospy.Publisher('speed_value', Int32, queue_size=10)
    rospy.init_node('processing', anonymous=True)
    rospy.Subscriber("imu_data", Vector3, callback)
    rospy.Subscriber("target",Int32,callback2)
    rospy.Subscriber('power',Bool,shut_down)
    rate = rospy.Rate(5) # 10hz
    # set_point=float(input())
    # pid = PID(0.5, 0.01, 0.1, setpoint=set_point)
    while not rospy.is_shutdown():
        # hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(f"Target {set_point} ,Current {current_value}, PWM {pwm_voltage}")
        pwm_voltage=pid(current_value)
        pub.publish(int(pwm_voltage))
        rate.sleep()
    pub.publish(0)

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
