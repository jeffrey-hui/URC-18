import smach_ros
import smach

from states import MarkGroundPos, DrillHole, ReturnToGroundPosition, MoveToSciencePos, LowerScienceSensors, \
    RaiseScienceSensors, WaitForScienceStabilize

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
                               "succeeded": 'RETURN_TO_GROUND',
                               "preempted": "preempted",
                               "aborted": "aborted"
                           })
    smach.StateMachine.add('RETURN_TO_GROUND', ReturnToGroundPosition(),
                           transitions={
                               "succeeded": 'MOVE_TO_SCIENCE',
                               "preempted": "preempted",
                               "aborted": "MARK_GROUND_POS"   # mark new ground pos
                           })
    smach.StateMachine.add('MOVE_TO_SCIENCE', MoveToSciencePos(),
                           transitions={
                               "succeeded": 'LOWER_SCIENCE_SENSORS',
                               "preempted": "preempted",
                               "aborted": "aborted"
                           })
    smach.StateMachine.add('LOWER_SCIENCE_SENSORS', LowerScienceSensors(),
                           transitions={
                               "succeeded": 'WAIT_STABILIZE',
                               "preempted": "preempted",
                               "aborted": "aborted"
                           })
    smach.StateMachine.add('WAIT_STABILIZE', WaitForScienceStabilize(),
                           transitions={
                               "succeeded": 'RAISE_SCIENCE_SENSORS',
                               "preempted": "preempted",
                               "aborted": "aborted"
                           })
    smach.StateMachine.add('RAISE_SCIENCE_SENSORS', RaiseScienceSensors(),
                           transitions={
                               "succeeded": 'succeeded',
                               "preempted": "preempted",
                               "aborted": "aborted"
                           })
