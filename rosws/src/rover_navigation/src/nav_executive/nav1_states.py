import rospy
from geometry_msgs.msg import PoseStamped, Pose, Quaternion
from smach import Sequence, State
from smach_ros import SimpleActionState
from rover_navigation.msg import Path
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction

pub_path = rospy.Publisher("/move_base/global_costmap/lines_layer/path", Path, queue_size=10)


class SendWaypointsState(State):
    def __init__(self):
        super(SendWaypointsState, self).__init__(io_keys=["waypoints", "goal_position"], outcomes=["goal"])

    def execute(self, ud):
        path_msg = Path(ud.waypoints)
        pub_path.publish(path_msg)
        return "goal"


class SendEmptyWaypointsState(State):
    def __init__(self):
        super(SendEmptyWaypointsState, self).__init__(io_keys=["goal_position"], outcomes=["ok"])

    def execute(self, ud):
        path_msg = Path([])
        pub_path.publish(path_msg)
        return "ok"


Nav1 = Sequence(outcomes=["goal", "fail"], connector_outcome="goal", input_keys=["goal_position", "waypoints"])
with Nav1:
    Sequence.add("SEND_WAYPOINTS", SendWaypointsState())
    Sequence.add("NAV", SimpleActionState(
        "/move_base",
        MoveBaseAction,
        goal_slots=["target_pose"]
    ), transitions={
        'succeeded': 'goal',
        'preempted': 'fail',
        'aborted': 'fail'
    }, remapping={
        "target_pose": "goal_position"
    })


def goal_cb(ud, _):
    pose = PoseStamped(
        ud.goal_position.header,
        Pose(
            ud.goal_position.point,
            Quaternion(0, 0, 0, 1)
        )
    )
    return MoveBaseGoal(pose)


Goto1 = Sequence(outcomes=["ok", "fail"], connector_outcome="ok", input_keys=["goal_position"])
with Goto1:
    Sequence.add("CLEAR_WAYPOINTS", SendEmptyWaypointsState())
    Sequence.add("GOTO", SimpleActionState(
        "/move_base",
        MoveBaseAction,
        goal_cb=goal_cb,
        input_keys=["goal_position"]
    ), transitions={
        'succeeded': 'ok',
        'preempted': 'fail',
        'aborted': 'fail'
    })
