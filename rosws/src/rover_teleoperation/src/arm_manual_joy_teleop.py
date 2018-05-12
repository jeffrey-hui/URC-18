#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64MultiArray
import geometry_msgs.msg

rospy.init_node("arm_manual_joy_teleop")

pub = rospy.Publisher("/arm_manual_controller/command", Float64MultiArray, queue_size=10)

def joyCB(dat):
    '''

    joystick data

    :type dat: Joy
    :return:
    '''

    rawvel_outer = -dat.axes[2] * 50

    rawvel_inner = 0
    if dat.buttons[4]:
        rawvel_inner -= 60
    if dat.buttons[5]:
        rawvel_inner += 60

    rawvel_tilt = -dat.axes[5] * 300
    pub.publish(data=[0, rawvel_outer, rawvel_inner, rawvel_tilt])


sub = rospy.Subscriber("joy", Joy, joyCB, queue_size=10)

rospy.spin()
