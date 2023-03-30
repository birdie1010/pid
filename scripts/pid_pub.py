#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32
from simple_pid import PID
from std_msgs.msg import Bool

set_point=0
enc_value=0
pwm_voltage=0
pid= PID(0,0,0,0)

def callback(data):
    global enc_value
    # rospy.loginfo(data.data)
    enc_value=(data.data)*360/100

def callback2(data1):
    global set_point,pid
    set_point=data1.data
    pid = PID(0.5, 0.01, 0.1, setpoint=set_point)

def shut_down(data2):
    rospy.loginfo(f"Shutdown!!!!")
    if(data2.data==0):
        rospy.signal_shutdown("Button pressed")  


def talker():
    global enc_value,set_point,pwm_voltage,pid
    pub = rospy.Publisher('speed_value', Int32, queue_size=10)
    rospy.init_node('processing', anonymous=True)
    rospy.Subscriber("current_angle", Int32, callback)
    rospy.Subscriber("target",Int32,callback2)
    rospy.Subscriber('power',Bool,shut_down)
    rate = rospy.Rate(5) # 10hz
    # set_point=float(input())
    # pid = PID(0.5, 0.01, 0.1, setpoint=set_point)
    while not rospy.is_shutdown():
        # hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(f"Target {set_point} ,Current {enc_value}, PWM {pwm_voltage}")
        pwm_voltage=pid(enc_value)
        pub.publish(int(pwm_voltage))
        rate.sleep()
    pub.publish(0)

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
