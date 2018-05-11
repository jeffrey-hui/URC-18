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

    rawvel = dat.axes[2]
    if dat.buttons[0]:
        pub.publish(data=[0, 0, rawvel, 0])
    elif dat.buttons[1]:
        pub.publish(data=[0, 0, 0, rawvel])
    else:
        pub.publish(data=[rawvel, rawvel, 0, 0])


sub = rospy.Subscriber("joy", Joy, joyCB, queue_size=10)

rospy.spin()
