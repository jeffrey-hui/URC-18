import rospy
from smach import Sequence, State
from smach_ros import SimpleActionState
from rover_navigation.msg import Path
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction


class SendWaypointsState(State):
    def __init__(self):
        super(SendWaypointsState, self).__init__(io_keys=["waypoints", "goal_position"], outcomes=["goal"])

    def execute(self, ud):
        path_msg = Path(ud.waypoints)
        pub = rospy.Publisher("/move_base/global_costmap/lines_layer/path", Path, queue_size=10)
        pub.publish(path_msg)
        pub.unregister()
        return "goal"


Nav1 = Sequence(outcomes=["goal", "fail"], connector_outcome="goal")
with Nav1:
    Sequence.add("SEND_WAYPOINTS", SendWaypointsState())
    Sequence.add("NAV", SimpleActionState(
        "/move_base",
        MoveBaseAction,
        goal_slots=["goal_position"]
    ), transitions={
        'succeeded': 'goal',
        'preempted': 'fail',
        'aborted': 'fail'
    })
