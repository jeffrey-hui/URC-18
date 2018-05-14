#!/usr/bin/env python
import rospy

rospy.init_node("nav_executive")

from smach import StateMachine, State
from smach_ros import ActionServerWrapper, IntrospectionServer
from rover_navigation.msg import RunNavAction

# todo: add proper preempt support

from nav_states_common import Nav1, Nav2
from nav1_states import Goto1
from search_state import SearchState

sm = StateMachine(
    outcomes=["ok", "fail", "preempted"], input_keys=["goal"]
)


class UnstufferState(State):
    def __init__(self):
        super(UnstufferState, self).__init__(outcomes=["ok"], input_keys=["goal"], output_keys=["waypoints", "goal_position"])

    def execute(self, ud):
        ud.waypoints = ud.goal.waypoints
        ud.goal_position = ud.goal.goal_position
        return "ok"


with sm:
    StateMachine.add("UNSTUFF", UnstufferState(), transitions={
        "ok": "NAV1"
    })
    StateMachine.add("NAV1", Nav1, transitions={
        "fail": "NAV2",
        "preempted": "preempted",
        "goal": "SEARCH",
        "tennis": "GOTO1"
    })
    StateMachine.add("NAV2", Nav2, transitions={
        "fail": "fail",
        "tennis": "GOTO1",
        "goal": "SEARCH",
        "preempted": "preempted"
    })
    StateMachine.add("GOTO1", Goto1, transitions={
        "ok": "ok",
        "preempted": "preempted",
        "fail": "fail"  # fixme: add goto2
    }, remapping={
        "goal_position": "tennis_position"
    })
    StateMachine.add("SEARCH", SearchState(), transitions={
        "go_back": "GOTO1",
        "tennis": "GOTO1",
        "preempted": "preempted"
    }, remapping={
        "tennis_position": "goal_position"
    })

isw = IntrospectionServer("nav_server", sm, "/NAV")
asw = ActionServerWrapper("/nav", RunNavAction, sm,
                          succeeded_outcomes=["ok"],
                          aborted_outcomes=["fail"],
                          goal_key="goal")

isw.start()
asw.run_server()

rospy.spin()