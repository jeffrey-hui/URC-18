#!/usr/bin/env python
import std_msgs.msg
import rospy

rospy.init_node("wclient")

p = rospy.Publisher("/ping", std_msgs.msg.Empty, queue_size=10)
r = rospy.Rate(5)

while not rospy.is_shutdown():
    r.sleep()
    p.publish(std_msgs.msg.Empty())

