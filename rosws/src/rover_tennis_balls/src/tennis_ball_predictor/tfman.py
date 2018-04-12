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
        self.buf_interface = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buf)
        self.fixed = fixed

    def get_collider_object(self, msg):
        """
        converts msg to an object from collider.py

        :type msg: rover_tennis_balls.msg.TennisBall
        """

        if msg.tennis_ball_type == msg.TENNIS_BALL_RAY:
            origin = geometry_msgs.msg.Vector3Stamped(
                msg.header,
                geometry_msgs.msg.Vector3(0, 0, 0)
            )
            origin = self.buf_interface.transform(
                origin, self.fixed, rospy.Duration.from_sec(0.2)
            )
            p_o_r_d = geometry_msgs.msg.Vector3Stamped(msg.header, msg.position_or_ray_dir)
            pos = self.buf_interface.transform(
               p_o_r_d, self.fixed, rospy.Duration.from_sec(0.2)
            )
            origin = np.array([
                origin.x, origin.y, origin.z
            ])
            pos = np.array([
                pos.x, pos.y, pos.z
            ])
            pos -= origin
            return collider.Ray(origin, pos, msg.confidence)
        else:
            pos = tf2_ros.Stamped(msg.position_or_ray_dir, msg.header.stamp, msg.header.frame_id)
            pos = self.buf_interface.transform(
                pos, self.fixed, rospy.Duration.from_sec(0.2)
            )
            return collider.Point(pos, msg.confidence)