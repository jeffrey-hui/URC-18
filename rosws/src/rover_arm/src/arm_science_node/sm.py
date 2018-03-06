import smach_ros
import smach
from actionlib import SimpleActionServer, rospy

from states import MarkGroundPos, DrillHole, ReturnToGroundPosition, MoveToSciencePos, LowerScienceSensors, \
    RaiseScienceSensors, WaitForScienceStabilize
from rover_arm.msg import ScienceArmAction, ScienceArmFeedback

sm = smach.StateMachine(outcomes=["succeeded", "aborted", "preempted"])


class ScienceActionServer:
    def __init__(self):
        self.as_ = SimpleActionServer("science_arm_action", ScienceArmAction)
        self.as_.register_goal_callback(self.goal)
        self.as_.register_preempt_callback(self.preempt)
        self.goal_running = False

    def goal(self):
        goal_ = self.as_.accept_new_goal()
        self.goal_running = True
        sm.execute()

    def feedback(self, str_):
        if self.goal_running:
            self.as_.publish_feedback(ScienceArmFeedback(currentAction=str_))

    def termination_cb(self, ud, ts, outcome):
        rospy.logdebug("ended")
        self.goal_running = False
        if outcome == "succeeded":
            self.as_.set_succeeded()
        elif outcome == "preempted":
            self.as_.set_preempted()
        else:
            self.as_.set_aborted()

    def preempt(self):
        sm.request_preempt()

    def go(self):
        self.as_.start()


sas = ScienceActionServer()
sas.go()


with sm:
    smach.StateMachine.add('MARK_GROUND_POS', MarkGroundPos(sas),
                           transitions={
                               "succeeded": 'DRILL_HOLE',
                               "preempted": "preempted",
                               "aborted": "aborted"
                           })
    smach.StateMachine.add('DRILL_HOLE', DrillHole(sas),
                           transitions={
                               "succeeded": 'RETURN_TO_GROUND',
                               "preempted": "preempted",
                               "aborted": "aborted"
                           })
    smach.StateMachine.add('RETURN_TO_GROUND', ReturnToGroundPosition(sas),
                           transitions={
                               "succeeded": 'MOVE_TO_SCIENCE',
                               "preempted": "preempted",
                               "aborted": "MARK_GROUND_POS"   # mark new ground pos
                           })
    smach.StateMachine.add('MOVE_TO_SCIENCE', MoveToSciencePos(sas),
                           transitions={
                               "succeeded": 'LOWER_SCIENCE_SENSORS',
                               "preempted": "preempted",
                               "aborted": "aborted"
                           })
    smach.StateMachine.add('LOWER_SCIENCE_SENSORS', LowerScienceSensors(sas),
                           transitions={
                               "succeeded": 'WAIT_STABILIZE',
                               "preempted": "preempted",
                               "aborted": "aborted"
                           })
    smach.StateMachine.add('WAIT_STABILIZE', WaitForScienceStabilize(sas),
                           transitions={
                               "succeeded": 'RAISE_SCIENCE_SENSORS',
                               "preempted": "preempted",
                               "aborted": "aborted"
                           })
    smach.StateMachine.add('RAISE_SCIENCE_SENSORS', RaiseScienceSensors(sas),
                           transitions={
                               "succeeded": 'succeeded',
                               "preempted": "preempted",
                               "aborted": "aborted"
                           })
