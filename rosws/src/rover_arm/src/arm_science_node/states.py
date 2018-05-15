import rospy
from smach import State
from std_msgs.msg import Float64MultiArray

import teleop

my_target = rospy.Publisher("/arm_manual_controller/command", Float64MultiArray, queue_size=10)


class MarkGroundPos(State):
    # noinspection PyTypeChecker
    def __init__(self):
        super(MarkGroundPos, self).__init__(outcomes=("succeeded", "aborted", "preempted"),
                                            output_keys=["arm_pos_tuple"])

    def execute(self, ud):
        teleop.Teleop.instance.enable_teleop()
        teleop.Teleop.instance.target_position[2] = 0.24
        teleop.Teleop.instance.wait_for_teleop_done()
        ud.arm_pos_tuple = teleop.Teleop.instance.target_position
        teleop.Teleop.instance.disable_teleop()
        return "succeeded"

    def request_preempt(self):
        super(MarkGroundPos, self).request_preempt()


class DrillHole(State):
    def __init__(self):
        super(DrillHole, self).__init__(outcomes=("succeeded", "aborted", "preempted"),
                                        input_keys=["arm_pos_tuple"],
                                        output_keys=["ground_pos_tuple"])

    def execute(self, ud):
        teleop.Teleop.instance.enable_teleop()
        teleop.Teleop.instance.enable_drill()
        teleop.Teleop.instance.lock_theta = True
        teleop.Teleop.instance.target_position = ud.arm_pos_tuple[:]
        teleop.Teleop.instance.wait_for_teleop_done()
        ud.ground_pos_tuple = ud.arm_pos_tuple[:]
        teleop.Teleop.instance.disable_teleop()
        return "succeeded"


class GoToTarget(State):
    def __init__(self, target, tdir, **kwargs):
        super(GoToTarget, self).__init__(**kwargs)
        self.target = target
        self.target_direction = tdir

    def execute(self, ud):
        target_pos = self.target(ud)
        teleop.Teleop.instance.disable_teleop()
        my_target.publish(data=target_pos[:])
        max_time = 30 * 30
        i = 0
        while abs(teleop.Teleop.instance.position[self.target_direction] - target_pos[self.target_direction]) > 0.02:
            # print("error: " + str(teleop.Teleop.instance.position[0]) + " < " + str(target_pos[0]) + ", i =" + str(i))
            teleop.Teleop.instance.rate.sleep()
            my_target.publish(data=target_pos[:])
            i += 1
            if i > max_time:
                my_target.publish(data=teleop.Teleop.instance.position)
                return "aborted"
        return "succeeded"


class ReturnToGroundPosition(GoToTarget):
    def __init__(self):
        super(ReturnToGroundPosition, self).__init__(lambda ud: ud.ground_pos_tuple, 0,
                                                     outcomes=("succeeded", "aborted", "preempted"),
                                                     input_keys=["ground_pos_tuple"],
                                                     output_keys=["ground_pos_tuple"])


class MoveToSciencePos(GoToTarget):
    def __init__(self):
        super(MoveToSciencePos, self).__init__(
            lambda ud: (ud.ground_pos_tuple[0], ud.ground_pos_tuple[1] + 0.261, ud.ground_pos_tuple[2]), 1,
            outcomes=("succeeded", "aborted", "preempted"),
            input_keys=["ground_pos_tuple"],
            output_keys=["science_pos_tuple"])

    def execute(self, ud):
        res = super(MoveToSciencePos, self).execute(ud)
        print("ok")
        ud.science_pos_tuple = teleop.Teleop.instance.position[:]
        return res


class LowerScienceSensors(GoToTarget):
    def __init__(self):
        super(LowerScienceSensors, self).__init__(
            lambda ud: (ud.science_pos_tuple[0], ud.science_pos_tuple[1], 0), 2,
            outcomes=("succeeded", "aborted", "preempted"),
            input_keys=["science_pos_tuple"],
            output_keys=["science_pos_tuple"])


class WaitForScienceStabilize(State):
    def __init__(self):
        super(WaitForScienceStabilize, self).__init__(
            outcomes=("succeeded", "aborted", "preempted"),
            input_keys=["science_pos_tuple"],
            output_keys=["science_pos_tuple"])

    def execute(self, ud):
        rospy.sleep(2)
        return "succeeded"


class RaiseScienceSensors(GoToTarget):
    def __init__(self):
        super(RaiseScienceSensors, self).__init__(
            lambda ud: (ud.science_pos_tuple[0], ud.science_pos_tuple[1], 0.27), 2,
            outcomes=("succeeded", "aborted", "preempted"),
            input_keys=["science_pos_tuple"],
            output_keys=["science_pos_tuple"])