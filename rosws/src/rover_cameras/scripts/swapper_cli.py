#!/usr/bin/env python
import rospy
import rover_cameras.srv
import rover_cameras.msg

rospy.init_node("swapper_cli")

load_camera = rospy.ServiceProxy("/swapper/load_camera", rover_cameras.srv.LoadCamera)
unload_camera = rospy.ServiceProxy("/swapper/unload_camera", rover_cameras.srv.UnloadCamera)

print "Camera MGMT v1"
print "? for help"

while not rospy.is_shutdown():
    i = raw_input("> ")
    shlex = i.strip()
    if shlex == "":
        continue
    elif shlex == "?":
        print "Commands:"
        print "load camera_name camera_url [unload] - load camera_name with device camera_url. (optionally unload camera unload)"
        print "unload camera_name - unload camera_name"
        print "list - list cameras"
        print "quit - quit"
    else:
        shlex = shlex.split(" ")
        try:
            if shlex[0] == "load":
                name = shlex[1]
                url = shlex[2]
                if len(shlex) == 4:
                    unload = shlex[3]
                else:
                    unload = ""
                load_camera(rover_cameras.srv.LoadCameraRequest(
                    camera_name=name, video_device=url, unload=unload
                ))
                print "Started camera {}".format(name)
            elif shlex[0] == "unload":
                name = shlex[1]
                if unload_camera(rover_cameras.srv.UnloadCameraRequest(name)).success:
                    print "Unloaded camera."
                else:
                    print "That camera does not exist."
            elif shlex[0] == "list":
                cameras = rospy.wait_for_message("/swapper/cameras", rover_cameras.msg.ActiveCameras)
                print "Active cameras:"
                for i in cameras.cameras:
                    print "- {}".format(i)
            elif shlex[0] == "quit":
                print "Goodbye"
                break
            else:
                print "Invalid command"
        except IndexError:
            print "Wrong number of arguments."