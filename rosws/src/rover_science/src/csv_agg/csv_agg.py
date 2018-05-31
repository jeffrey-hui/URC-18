#!/usr/bin/env python
import rospy
import std_msgs.msg
import std_srvs.srv
import time
import csv

headers = (
    "Time",
    "No#",
    "pH",
    "EC_Temp",
    "EC_Value",
    "Soil Humidity",
    "Soil Temp",
    "Dust Density",
    "GeigeCPM",
    "Wind Velocity",
    "DO"
)

current_rows = [headers]

last_data = None
last_do = None
cur_count = 0

rospy.init_node("csv_agg")


def save_data():
    with open("~/urc18_science_data.csv", "w") as f:
        w = csv.writer(f)
        w.writerows(current_rows)
    rospy.loginfo("Wrote {} rows to disk at ~/urc18_science_data.csv".format(cur_count+1))


def service_handler(req):
    global cur_count
    data_1 = rospy.wait_for_message("/science/data", std_msgs.msg.Float32MultiArray)
    row = [time.strftime("%a, %d %b %Y %H:%M:%S"), cur_count]
    cur_count += 1
    row.extend(data_1.data[:])
    row.append(rospy.wait_for_message("/science/do", std_msgs.msg.Float32).data)
    current_rows.append(row)
    rospy.loginfo("Recorded measurement!")
    return std_srvs.srv.EmptyResponse()


serv = rospy.Service("/science/record", std_srvs.srv.Empty, service_handler)
rospy.loginfo("csv_agg started!")

rospy.spin()
