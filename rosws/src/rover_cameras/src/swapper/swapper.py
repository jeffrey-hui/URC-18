#!/usr/bin/env python
import rospy

rospy.init_node("swapper")
import node_controller
import rover_cameras.srv
import rover_cameras.msg

pub = rospy.Publisher("~cameras", rover_cameras.msg.ActiveCameras, queue_size=10, latch=True)


def pub_cameras():
    m = rover_cameras.msg.ActiveCameras()
    m.cameras = node_controller.RUNNING_CAMERAS[:]
    pub.publish(m)


def on_load_camera(req):
    preffered_slot = req.unload if req.unload != "" else None

    node_controller.start_camera(req.video_device, req.camera_name, preffered_slot)
    rospy.loginfo("Starting camera.")
    pub_cameras()
    return rover_cameras.srv.LoadCameraResponse()


def on_unload_camera(req):
    rospy.loginfo("Stopping camera")
    pub_cameras()
    return rover_cameras.srv.UnloadCameraResponse(node_controller.stop_camera(req.camera_name))


pub_cameras()
rospy.loginfo("Starting!")
node_controller.start_camera("/dev/video0", "head_camera")
node_controller.start_camera("/dev/video1", "belly_camera")

load_cam = rospy.Service("~load_camera", rover_cameras.srv.LoadCamera, on_load_camera)
unload_cam = rospy.Service("~unload_camera", rover_cameras.srv.UnloadCamera, on_unload_camera)

rospy.spin()
node_controller.stop_all()
