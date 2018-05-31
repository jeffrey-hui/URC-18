#!/usr/bin/env python
import math
import std_msgs.msg
import std_srvs.srv
import sensor_msgs.msg
import rospy

LEFT_AXIS = 1
RIGHT_AXIS = 4

is_enabled = True
rospy.loginfo("Starting drive joy teleoperation node")
rospy.init_node("simple_drive_joy_node")
drive_control_left = rospy.Publisher("/left_wheels_controller/cmd", std_msgs.msg.Float64, queue_size=20)
drive_control_right = rospy.Publisher("/right_wheels_controller/cmd", std_msgs.msg.Float64, queue_size=20)

speed_value = 3


def toggle_enabled(resp):
    global is_enabled
    if resp.data:
        is_enabled = not is_enabled
    if not is_enabled:
        drive_control_left.publish(0.0)
        drive_control_right.publish(0.0)
    return std_srvs.srv.SetBoolResponse(success=is_enabled)


enable_service = rospy.Service("~set_enabled", std_srvs.srv.SetBool, toggle_enabled)

def ctrl_curve(x):
    return math.copysign(x**2, x) * speed_value


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
            speed_value += tri_state * 5
            speed_value = max(1.175, min(40.75, speed_value))

    if is_enabled:
        drive_control_right.publish(left)
        drive_control_left.publish(right)


joy_listener = rospy.Subscriber("joy", sensor_msgs.msg.Joy, queue_size=20, callback=on_joy_data)
rospy.spin()
