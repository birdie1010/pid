#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import Vector3
from std_msgs.msg import Int32
from std_msgs.msg import Bool


set_point_x=0
accelx=0
accely=0
accelz=0


def callback(data):
    global accelx,accely,accelz
    accelx=data.x
    accely=data.y
    accelz=data.z
    ori_calc()

def ori_calc():
    global accelx,accely,accelz,set_point_x
    set_point_x=(math.atan2(-accelx,math.sqrt((accely**2)+(accelz**2))))*180/3.14    

def shut_down(data1):
    rospy.loginfo(f"Shutdown!!!!")
    if(data1.data==0):
        rospy.signal_shutdown("Button pressed")  





def talker():
    global set_point,accelx,accely,accelz
    pub = rospy.Publisher('current_angle', Int32, queue_size=10)
    rospy.init_node('processing', anonymous=True)
    rospy.Subscriber("imu_data", Vector3, callback)
    rospy.Subscriber('power',Bool,shut_down)
    rate = rospy.Rate(2) # 10hz
    # set_point=float(input())
    # pid = PID(0.5, 0.01, 0.1, setpoint=set_point)
    while not rospy.is_shutdown():
        # hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(f"Accelaration value {accelx}  {accely}  {accelz}     Current angle {set_point_x}")
        pub.publish(int(set_point_x))
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
