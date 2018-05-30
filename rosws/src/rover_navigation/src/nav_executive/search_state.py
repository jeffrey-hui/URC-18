import rospy
from geometry_msgs.msg import PointStamped
from smach import State


class SearchState(State):
    def __init__(self):
        super(SearchState, self).__init__(outcomes=["tennis", "go_back", "preempted"], output_keys=["tennis_position"], input_keys=["goal_position"])

    def execute(self, ud):
        if self.preempt_requested():
            return "preempted"

        rospy.logwarn("SearchState is not implemented, need an algo to explore for tennis balls. Skipping")
        ud.tennis_position = PointStamped(ud.goal_position.header, ud.goal_position.pose.position)

        return "go_back"
