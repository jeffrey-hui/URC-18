#!/usr/bin/env python
import rospy
import rover_tennis_balls.msg
import tfman
import collider
import geometry_msgs.msg


rospy.init_node("tennis_ball_predictor")

fixed_frame = rospy.get_param("~fixed_frame", "map")  # frame to publish tennis balls in (shouldn't move over time)
keep = rospy.get_param("~keep_measurements", 50)

tf_man = tfman.TfMan(fixed_frame)

measurements = []


def add_m(collide_t):
    global measurements
    if len(measurements) == keep:
        measurements.pop(0)
    measurements.append(collide_t)


pub = rospy.Publisher("tennis_ball_pos", rover_tennis_balls.msg.TennisBallPrediction, queue_size=5)


def make_prediction():
    rays = [x for x in measurements if isinstance(x, collider.Ray)]
    points = [x for x in measurements if isinstance(x, collider.Point)]

    pred = collider.calc_min_f(rays, points)
    if not pred.success:
        rospy.logwarn("Failed to converge on a prediction!")
    else:
        print(pred.cost)
        conf = min(1, len(measurements) / max(1, pred.cost + 1))
        rospy.loginfo("Tennis ball prediction w/ conf={}, pos={}".format(
            conf, pred.x
        ))
        msg = rover_tennis_balls.msg.TennisBallPrediction()
        msg.header.frame_id = fixed_frame
        msg.confidence = conf
        msg.tennis_ball_position = geometry_msgs.msg.Point(*pred.x)
        pub.publish(msg)


def on_new_measurement(m):
    add_m(tf_man.get_collider_object(m))
    make_prediction()


sub = rospy.Subscriber("tennis_ball_measurements", rover_tennis_balls.msg.TennisBall, callback=on_new_measurement, queue_size=5)

rospy.spin()
