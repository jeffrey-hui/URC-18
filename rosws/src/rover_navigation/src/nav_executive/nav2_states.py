import rospy
from smach import State


# todo: yell at adam and jeff to finish nav2

class Nav2(State):
    def execute(self, ud):
        rospy.logerr("NAV2 is not implemented. Yell at jeff/adam, not me!")
        return "fail"  # notimplementedtrap

    def __init__(self):
        super(Nav2, self).__init__(outcomes=("goal", "fail"), input_keys=["goal_position", "waypoints"])