import smach_ros
import smach

sm = smach.StateMachine(outcomes=["succeeded", "aborted", "preempted"])