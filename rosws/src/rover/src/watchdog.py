#!/usr/bin/env python
import rospy
import geometry_msgs.msg
import std_msgs.msg

rospy.init_node("watchdog_server")
last_ok = rospy.Time.now()

rospy.loginfo("Starting watchdog")

ZERO_MSGS = {
        "/left_wheels_controller/cmd": std_msgs.msg.Float64(0.0),
        "/right_wheels_controller/cmd": std_msgs.msg.Float64(0.0),
        "/arm_manual_controller/command": std_msgs.msg.Float64MultiArray(data=[0, 0, 0, 0, 0, 0])
}

ZERO_PUBS = {
        rospy.Publisher(k, v.__class__, queue_size=10): v for k, v in ZERO_MSGS
}

def reset_cb(msg):
    last_ok = rospy.Time.now()

def zero():
    for p, m in ZERO_PUBS:
        p.publish(m)

sub = rospy.Subscriber("/ping", std_msgs.msg.Empty, reset_cb, queue_size=10)

while not rospy.is_shutdown():
    if (rospy.Time.now() - last_ok).to_sec() > 1.0:
        rospy.logwarn("Lost connection! Zeroing all registered messages.")
        zero()
    rospy.sleep(0.5)
