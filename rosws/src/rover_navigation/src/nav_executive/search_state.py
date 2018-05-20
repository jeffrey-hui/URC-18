import rospy
from geometry_msgs.msg import PointStamped
from smach import State


class SearchState(State):
    def __init__(self):
        super(SearchState, self).__init__(outcomes=["tennis", "go_back", "preempted"], output_keys=["goal_position"], input_keys=["waypoints"])

    def execute(self, ud):
        if self.preempt_requested():
            return "preempted"

        rospy.logwarn("SearchState is not implemented, need an algo to explore for tennis balls. Skipping")
        goal_position = ud.waypoints[-1]
        ud.goal_position = goal_position
        print(goal_position)

        return "go_back"
