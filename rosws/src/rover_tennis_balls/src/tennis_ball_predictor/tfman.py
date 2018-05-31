import numpy as np

import rospy
import geometry_msgs.msg
import rover_tennis_balls.msg
import tf2_ros
import tf2_geometry_msgs

import collider


class TfMan:
    def __init__(self, fixed):
        self.buf = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buf)
        self.fixed = fixed

    def get_collider_object(self, msg):
        """
        converts msg to an object from collider.py

        :type msg: rover_tennis_balls.msg.TennisBall
        """

        if msg.tennis_ball_type == msg.TENNIS_BALL_RAY:
            origin = geometry_msgs.msg.PoseStamped(
                msg.header,
                geometry_msgs.msg.Pose(
                    geometry_msgs.msg.Vector3(0, 0, 0),
                    geometry_msgs.msg.Quaternion(0, 0, 0, 1)
                )
            )
            origin = self.buf.transform(
                origin, self.fixed, rospy.Duration.from_sec(0.2)
            )
            p_o_r_d = geometry_msgs.msg.PoseStamped(
                msg.header,
                geometry_msgs.msg.Pose(
                    msg.position_or_ray_dir,
                    geometry_msgs.msg.Quaternion(0, 0, 0, 1)
                )
            )
            pos = self.buf.transform(
               p_o_r_d, self.fixed, rospy.Duration.from_sec(0.2)
            )
            origin = np.array([
                origin.pose.position.x, origin.pose.position.y, origin.pose.position.z
            ])
            pos = np.array([
                pos.pose.position.x, pos.pose.position.y, pos.pose.position.z
            ])
            pos -= origin
            return collider.Ray(origin, pos, msg.confidence)
        else:
            pos = geometry_msgs.msg.PoseStamped(
                msg.header,
                geometry_msgs.msg.Pose(
                    msg.position_or_ray_dir,
                    geometry_msgs.msg.Quaternion(0, 0, 0, 1)
                )
            )
            pos = self.buf.transform(
                pos, self.fixed, rospy.Duration.from_sec(0.2)
            )
            pos = np.array([
                pos.pose.position.x, pos.pose.position.y, pos.pose.position.z
            ])
            return collider.Point(pos, msg.confidence)
