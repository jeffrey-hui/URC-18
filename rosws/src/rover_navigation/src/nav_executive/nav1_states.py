import rospy
from geometry_msgs.msg import PoseStamped, Pose, Quaternion
from smach import Sequence, State
from smach_ros import SimpleActionState
from rover_navigation.msg import Path, GotoPointGoal, GotoPointAction


def goal_cb1(ud, _):
    m = Path(ud.waypoints)
    return GotoPointGoal(m)


Nav1 = Sequence(["goal", "fail", "preempted"], "ok", ["waypoints"])
Goto1 = Sequence(["ok", "fail", "preempted"], "ok", ["goal_position"])

NAV1 = SimpleActionState(
    "/navigator",
    GotoPointAction,
    goal_cb=goal_cb1,
    input_keys=["waypoints"]
)


def goal_cb2(ud, _):
    m = Path([ud.goal_position])
    return GotoPointGoal(m)


GOTO1 = SimpleActionState(
    "/navigator",
    GotoPointAction,
    goal_cb=goal_cb2,
    input_keys=["goal_position"])

with Nav1:
    Sequence.add("NAV1", NAV1, transitions={
        "succeeded": "goal",
        "preempted": "preempted",
        "aborted": "fail"
    }, remapping={"waypoints": "waypoints"})

with Goto1:
    Sequence.add("GOTO1", GOTO1, transitions={
        "succeeded": "ok",
        "preempted": "preempted",
        "aborted": "fail"
    }, remapping={"goal_position": "goal_position"})