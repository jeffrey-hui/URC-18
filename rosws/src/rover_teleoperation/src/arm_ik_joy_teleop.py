#!/usr/bin/env python
import rospy
from rover_ik.srv import RequestPositionRequest, RequestPositionResponse, RequestPosition
from sensor_msgs.msg import Joy
import geometry_msgs.msg

rospy.init_node("arm_ik_joy_teleop")

serv = rospy.ServiceProxy("/arm_ik_controller/request_position", RequestPosition)

velocity = [
    0, 0, 0
]


def joyCB(dat):
    '''

    joystick data

    :type dat: Joy
    :return:
    '''
    global velocity
    velocity[0] = dat.axes[1] / 4.0
    velocity[2] = dat.axes[5] / 20.0
    velocity[1] = dat.axes[0] / 4.0

sub = rospy.Subscriber("joy", Joy, joyCB, queue_size=10)
pub = rospy.Publisher("/arm_ik_controller/target", geometry_msgs.msg.Pose, queue_size=10)

rate = rospy.Rate(15)
t = rospy.Time.now()

position = [0, 0, 0]
rpy = [0, 0, 0, 0]

pose = serv(RequestPositionRequest()).current  # type: geometry_msgs.msg.Pose
position = [pose.position.x, pose.position.y, pose.position.z]
rpy = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]

while not rospy.is_shutdown():
    rate.sleep()
    t2 = rospy.Time.now()
    d = t2 - t
    secs = d.to_sec()

    x, y, z = [val * secs for val in velocity]
    position[0] += x
    position[1] += y
    position[2] += z
    print position
    p = geometry_msgs.msg.Pose()
    p.position.x, p.position.y, p.position.z = position
    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = rpy
    pub.publish(p)
    t = t2
