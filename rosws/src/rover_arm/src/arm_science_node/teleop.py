import rospy
import threading
from sensor_msgs.msg import Joy, JointState
from std_msgs.msg import Float64, Float64MultiArray

DRILL_POWER = 0.25
Y_SPEED = 0.25 / 30.0


class Teleop(threading.Thread):
    def __init__(self):
        super(Teleop, self).__init__()

        self.joy_sub = rospy.Subscriber("/joy", Joy, queue_size=10, callback=self.on_joy)
        self.joint_state_sub = rospy.Subscriber("/joint_states", JointState, queue_size=2, callback=self.on_joint_state)
        self.position = None
        self.needs_stop = False
        self.drill_on = False
        self.teleop_on = False
        self.teleop_done = threading.Condition()

        self.last_joy_state = None  # type: Joy
        self.joy_state = None
        self.drill_pub = rospy.Publisher("/drill_controller/command", Float64, queue_size=10)
        self.target_pub = rospy.Publisher("/arm_manual_controller/command", Float64MultiArray, queue_size=10)
        self.rate = rospy.Rate(30)

        self.target_position = None

    def on_joint_state(self, m):
        self.position = [
            max(0, m.position[m.name.index("arm_slide_pole_to_arm_slider_unit")]),
            m.position[m.name.index("arm_slider_unit_to_arm_xy_pole")],
            max(0, m.position[m.name.index("science_crossbar_to_science_sensors")])
        ]

    def on_joy(self, m):
        self.last_joy_state = self.joy_state
        self.joy_state = m

    def enable_drill(self):
        self.enable_teleop()
        self.drill_on = True

    def enable_teleop(self):
        self.teleop_on = True
        while self.position is None:
            pass
        self.target_position = self.position[:]

    def disable_teleop(self):
        self.drill_on = False
        self.teleop_on = False

    def wait_for_teleop_done(self):
        with self.teleop_done:
            self.teleop_done.wait()

    def stop(self):
        self.needs_stop = True

    def falling_edge(self, i):
        if self.last_joy_state is None:
            return False
        if self.last_joy_state.buttons[i] and not self.joy_state.buttons[i]:
            return True
        return False

    def run(self):
        while not self.needs_stop:
            self.rate.sleep()
            if self.joy_state is not None:
                if self.drill_on:
                    self.drill_pub.publish(DRILL_POWER * int(self.joy_state.buttons[0]))
                    print DRILL_POWER * int(self.joy_state.buttons[0])
                if self.teleop_on:
                    y_offset = self.joy_state.axes[1] * Y_SPEED
                    self.target_position[0] += y_offset
                    print self.target_position

                    self.target_pub.publish(Float64MultiArray(data=self.target_position))
