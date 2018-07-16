import math
from threading import Lock

from std_msgs.msg import Header

from geometry_msgs.msg import PointStamped, PoseStamped, Point
from nav_msgs.msg import Odometry
from smach import Concurrence, State
import rospy
from rover_tennis_balls.msg import TennisBallPrediction
import tf

import nav2_states
import nav1_states


class TennisBallMonitoringState(State):
    TENNIS_BALL_PROXIMITY = 30 # in meters, wait until this close
    TENNIS_BALL_MIN_CONFIDENCE = 0.5

    def __init__(self, topic_name):
        super(TennisBallMonitoringState, self).__init__(outcomes=["tennis", "fail"], input_keys=["goal_position"], output_keys=["tennis_position"])

        self.sub = None
        self.topic_name = topic_name
        self.most_recent_message = None  # type: TennisBallPrediction
        self.processing_lock = Lock()

    def _sub_callback(self, msg):
        """
        Subscriber callback

        :type msg: TennisBallPrediction
        """
        with self.processing_lock:
            self.most_recent_message = msg if msg.confidence > TennisBallMonitoringState.TENNIS_BALL_MIN_CONFIDENCE else None

    def execute(self, ud):
        if self.sub is not None:
            self.sub.unregister()
        self.sub = rospy.Subscriber(self.topic_name, TennisBallPrediction, callback=self._sub_callback, queue_size=10)
        self.most_recent_message = None  # type: TennisBallPrediction
        most_recent_time = -1
        tf_ = tf.TransformListener()
        goal_position = ud.goal_position # type: PointStamped
        ud.tennis_position = None  # if preempt, leave this null

        while not self.preempt_requested():
            if self.most_recent_message is None:
                rospy.sleep(0.5)
            else:
                if self.most_recent_message.header.stamp.to_sec() == most_recent_time:
                    rospy.sleep(0.5)
                else:
                    with self.processing_lock:
                        most_recent_time = self.most_recent_message.header.stamp.to_sec()
                        try:
                            current_robot_odometry = rospy.wait_for_message("/odometry/filtered", Odometry, timeout=1)  # type: Odometry
                        except rospy.ROSException:
                            rospy.logwarn("Couldn't get odom info for tennis tracker monitor state")
                            continue

                        current_robot_odometry = tf_.transformPose(goal_position.header.frame_id, PoseStamped(Header(frame_id=current_robot_odometry.child_frame_id), current_robot_odometry.pose.pose))
                        dist = math.sqrt(
                            (current_robot_odometry.pose.position.x - goal_position.pose.position.x)**2 +
                            (current_robot_odometry.pose.position.y - goal_position.pose.position.y)**2 +
                            (current_robot_odometry.pose.position.z - goal_position.pose.position.z)**2)

                        if dist < TennisBallMonitoringState.TENNIS_BALL_PROXIMITY:
                            ud.tennis_position = PointStamped(
                                self.most_recent_message.header,
                                self.most_recent_message.tennis_ball_position
                            )
                            rospy.loginfo("Found tennis ball!")
                            break

        if self.preempt_requested():
            return "fail"

        return "tennis"


def make_nav_sm(nav_state, tennis_topic):
    """
    Make a NAV<x> state.

    :param nav_state: the state

    nav_state takes in ud called goal_position and waypoints
    it has two outcomes: goal, fail, preempted
    preempt is called when tennis is found, make sure to implement it

    goal_position is a pose (for added laziness in move_base)

    :return: the state machine (actually a Concurrence but it doesn't matter)
    """

    def i_dont_care_callback(_):
        return True # whichever finishes first! (although tennis never finishes unless preempt...)

    # noinspection PyTypeChecker
    sm = Concurrence(outcomes=["tennis", "goal", "fail", "preempted"], default_outcome="fail", input_keys=["goal_position", "waypoints"], output_keys=["tennis_position"],
                     outcome_map={
                         "tennis": {"TENNIS_MONITOR": "tennis"},
                         "fail": {"NAV": "fail",
                                  "TENNIS_MONITOR": "fail"},
                         "goal": {"NAV": "goal"},
                         "preempted": {"NAV": "preempted",
                                       "TENNIS_MONITOR": "fail"}
                     },  child_termination_cb=i_dont_care_callback)

    with sm:
        Concurrence.add("TENNIS_MONITOR", TennisBallMonitoringState(tennis_topic), remapping={
            "tennis_position": "tennis_position"
        })
        Concurrence.add("NAV", nav_state)

    return sm


Nav2 = make_nav_sm(nav2_states.Nav2(), "/tennis_ball_map/tennis_ball_pos")  # fixme: add tennis_ball_gpsonly
Nav1 = make_nav_sm(nav1_states.Nav1, "/tennis_ball_map/tennis_ball_pos")