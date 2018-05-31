#!/usr/bin/env python
import os
import rospy

rospy.init_node("proxy_node")
os.system("docker start mapviz_proxy")
rospy.spin()
os.system("docker stop mapviz_proxy")