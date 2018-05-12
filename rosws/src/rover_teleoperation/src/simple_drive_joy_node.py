#!/usr/bin/env python
import math
import std_msgs.msg
import sensor_msgs.msg
import rospy

LEFT_AXIS = 1
RIGHT_AXIS = 4

rospy.loginfo("Starting drive joy teleoperation node")
rospy.init_node("simple_drive_joy_node")
drive_control_left = rospy.Publisher("/left_wheels_controller/cmd", std_msgs.msg.Float64, queue_size=20)
drive_control_right = rospy.Publisher("/right_wheels_controller/cmd", std_msgs.msg.Float64, queue_size=20)

speed_value = 1.175


def ctrl_curve(val):
    return val * speed_value


push_button_last = False


def on_joy_data(msg):
    global push_button_last
    global speed_value
    # type: (sensor_msgs.msg.Joy) -> None

    right = ctrl_curve(msg.axes[LEFT_AXIS])
    left = ctrl_curve(msg.axes[RIGHT_AXIS])

    tri_state = int(msg.axes[7])
    if tri_state == 0:
        push_button_last = False
    else:
        if not push_button_last:
            speed_value += tri_state
            speed_value = max(1.175, min(4.75, speed_value))
    drive_control_right.publish(left)
    drive_control_left.publish(-right)

joy_listener = rospy.Subscriber("joy", sensor_msgs.msg.Joy, queue_size=20, callback=on_joy_data)
rospy.spin()
