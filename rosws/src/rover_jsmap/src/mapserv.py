#!/usr/bin/env python
import rospy
import rospkg
import SimpleHTTPServer
import SocketServer
import os

rospy.init_node("jsmap_server")

rospkger = rospkg.RosPack()
pack_path = rospkger.get_path("rover_jsmap")
os.chdir(os.path.join(pack_path, "src"))

PORT = 8100

Handler = SimpleHTTPServer.SimpleHTTPRequestHandler

httpd = SocketServer.TCPServer(("", PORT), Handler)

rospy.loginfo("Starting server at port " + str(PORT))
httpd.serve_forever()