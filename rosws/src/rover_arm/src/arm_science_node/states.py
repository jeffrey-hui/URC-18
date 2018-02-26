from smach import State
import teleop


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
        teleop.Teleop.instance.target_position = ud.arm_pos_tuple
        teleop.Teleop.instance.wait_for_teleop_done()
        ud.ground_pos_tuple = ud.arm_pos_tuple
        teleop.Teleop.instance.disable_teleop()
        return "succeeded"
