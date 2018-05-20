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

    z = dat.axes[1] * 1000
    rawvel_outer = -dat.axes[2] * 200

    rawvel_inner = 0
    if dat.buttons[2]:
        rawvel_inner -= 500
    if dat.buttons[3]:
        rawvel_inner += 500

    rawvel_tilt = -dat.axes[5] * 300
    spin = 0
    if dat.buttons[4]:
        spin -= 200
    if dat.buttons[5]:
        spin += 200

    gripper_speed = 0
    if dat.buttons[0]:
        gripper_speed -= 300
    if dat.buttons[1]:
        gripper_speed += 300

    pub.publish(data=[z, rawvel_outer, rawvel_inner, rawvel_tilt, spin, gripper_speed])


sub = rospy.Subscriber("joy", Joy, joyCB, queue_size=10)

rospy.spin()
