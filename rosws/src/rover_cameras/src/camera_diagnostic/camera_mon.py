#!/usr/bin/env python
import rospy
import os
import diagnostic_updater
from diagnostic_msgs.msg import DiagnosticStatus

rospy.init_node("camera_mon")
expected_camera_nums = rospy.get_param("~expected_nums", [0, 1, 2])
if type(expected_camera_nums) is not list:
    rospy.logfatal("~expected_nums should be int list")
    exit(1)


def get_running_nums():
    directory_listing = os.listdir("/dev")
    video_paths = [x for x in directory_listing if x.startswith("video")]

    return [int(x[len("video"):]) for x in video_paths]


def running_task(stat):
    running_nums = get_running_nums()
    if sorted(running_nums) == expected_camera_nums:
        stat.summary(DiagnosticStatus.OK, "All cameras are connected.")
    else:
        stat.summary(DiagnosticStatus.ERROR, "Some cameras are not connected.")
    for i in expected_camera_nums:
        stat.add("/dev/video{}".format(i), "ok" if i in running_nums else "not ok")
    return stat


updater = diagnostic_updater.Updater()
updater.add("Cameras", running_task)

rate = rospy.Rate(0.25)
while not rospy.is_shutdown():
    rate.sleep()

    updater.update()
