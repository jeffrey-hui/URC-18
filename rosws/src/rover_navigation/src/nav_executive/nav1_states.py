import rospy
from geometry_msgs.msg import PoseStamped, Pose, Quaternion
from smach import Sequence, State
from smach_ros import SimpleActionState
from rover_navigation.msg import Path, GotoPointGoal, GotoPointAction


def goal_cb1(ud, _):
    m = Path(ud.waypoints)
    return GotoPointGoal(m)


Nav1 = SimpleActionState(
    "/navigator",
    GotoPointAction,
    goal_cb=goal_cb1,
    goal_slots=["target_pose"]
)


def goal_cb2(ud, _):
    m = Path([ud.goal_position])
    return GotoPointGoal(m)


Goto1 = SimpleActionState(
    "/navigator",
    MoveBaseAction,
    goal_cb=goal_cb,
    input_keys=["goal_position"])
