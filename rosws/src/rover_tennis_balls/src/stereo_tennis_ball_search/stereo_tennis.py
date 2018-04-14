#!/usr/bin/env python
import numpy as np
import rospy
import cv_bridge
import sensor_msgs.msg
from rover_tennis_balls.msg import TennisBall
from rover_tennis_balls.tennis import find_tennis_ball
from geometry_msgs.msg import Vector3
import projmath

rospy.init_node("monocular_tennis_ball_search")
cam_info = rospy.wait_for_message("camera/camera_info",
                                  sensor_msgs.msg.CameraInfo)  # type: sensor_msgs.msg.CameraInfo

proj_mat = np.array(cam_info.P, dtype=np.float32)
proj_mat = proj_mat.reshape((3, 4))
proj_mat = proj_mat[:, :3]

pos_tolerance = rospy.get_param("~pos_tolerance", 10)
size_tolerance = rospy.get_param("~size_tolerance", 15)

min_depth = rospy.get_param("~min_depth", 0.1)
max_depth = rospy.get_param("~max_depth", 2)

last_tennis_ball = None
frames = 0
decay = 0

cvb = cv_bridge.CvBridge()


def publish_tennis_ball(ball):
    global last_tennis_ball, frames, decay
    last_tennis_ball = ball
    frames += 1
    decay = 10
    ray = projmath.get_ray(ball[0], ball[1], proj_mat)
    confidence = min(1, (frames / 35) ** 2)
    confidence -= (10 - decay) / 30
    if ball[2] < 20:
        confidence -= 0.25
    elif ball[2] > 300:
        confidence -= 0.25

    try:
        depth_image = rospy.wait_for_message("camera/depth", sensor_msgs.msg.Image, 0.2)
    except rospy.ROSException:
        rospy.logerr("Couldn't get a depth image in time! Skipping this measurement")
        return

    cv_mat_depth = cvb.imgmsg_to_cv2(depth_image)
    depth = cv_mat_depth[int(ball[0])][int(ball[1])]
    if min_depth <= depth <= max_depth:
        rospy.loginfo("Publishing tennis ball as point")
        confidence = max(0.1, confidence)
        msg = TennisBall()
        msg.confidence = confidence
        msg.tennis_ball_type = msg.TENNIS_BALL_POS
        msg.header.frame_id = cam_info.header.frame_id
        msg.position_or_ray_dir = Vector3(*(ray * depth))
        pub.publish(msg)
    else:
        rospy.logwarn("Depth data made no sense, so publishing as a ray with low confidence")
        confidence = max(0.1, confidence-0.3)
        msg = TennisBall()
        msg.confidence = confidence
        msg.tennis_ball_type = msg.TENNIS_BALL_RAY
        msg.header.frame_id = cam_info.header.frame_id
        msg.position_or_ray_dir = Vector3(*ray)
        pub.publish(msg)


def on_image(msg):
    global last_tennis_ball, frames, decay
    cv_mat = cvb.imgmsg_to_cv2(msg, desired_encoding="bgr8")
    ball = find_tennis_ball(cv_mat)
    if ball is None:
        if decay > 0:
            decay -= 1
            return
        else:
            decay = 0
            frames = 0
            last_tennis_ball = None
    else:
        if last_tennis_ball is None and frames < 10:
            frames += 1
            return
        elif last_tennis_ball is None:
            last_tennis_ball = ball
            publish_tennis_ball(ball)
        frames += 1
        decay = 10
        x, y, radius = ball
        x_, y_, radius_ = last_tennis_ball
        d_x = x_ - x
        d_y = y_ - y
        d_r = abs(radius_ - radius)
        d_d = np.sqrt(d_x ** 2 + d_y ** 2)
        if d_r > size_tolerance or d_d > pos_tolerance:
            publish_tennis_ball(ball)


sub = rospy.Subscriber("camera/image", sensor_msgs.msg.Image, callback=on_image, queue_size=30)
pub = rospy.Publisher("tennis_ball_measurements", TennisBall, queue_size=10)

rospy.spin()
