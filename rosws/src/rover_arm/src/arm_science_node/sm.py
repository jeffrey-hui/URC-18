import smach_ros
import smach

from states import MarkGroundPos, DrillHole

sm = smach.StateMachine(outcomes=["succeeded", "aborted", "preempted"])


def start_action_server():
    pass


with sm:
    smach.StateMachine.add('MARK_GROUND_POS', MarkGroundPos(),
                           transitions={
                               "succeeded": 'DRILL_HOLE',
                               "preempted": "preempted",
                               "aborted": "aborted"
                           })
    smach.StateMachine.add('DRILL_HOLE', DrillHole(),
                           transitions={
                               "succeeded": 'succeeded',
                               "preempted": "preempted",
                               "aborted": "aborted"
                           })

