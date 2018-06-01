import subprocess
from signal import SIGINT

import rospy


RESOLUTION = [640, 480]
RESOLUTION_ARGS = ["_image_width:={}".format(RESOLUTION[0]), "_image_height:={}".format(RESOLUTION[1])]


class RunningImage:
    def __init__(self, camera_url, camera_name):
        self.name = camera_name
        self.run_str = ["rosrun", "usb_cam", "usb_cam_node", "_camera_name:=" + camera_name, "_video_device:=" + camera_url] + RESOLUTION_ARGS + ["__name:=" + camera_name]
        self._set_params()
        self.process = subprocess.Popen(self.run_str)

    def _set_params(self):
        rospy.set_param("/{}/image_raw/compressed/jpeg_quality".format(self.name), 15)

    def shutdown(self):
        self.process.send_signal(SIGINT)
        self.process.wait()


NODE_SLOTS = []
RUNNING_CAMERAS = []


def start_camera(camera_url, camera_name, preferred_slot=None):
    global NODE_SLOTS, RUNNING_CAMERAS
    if len(NODE_SLOTS) == 2:
        if preferred_slot == -1:
            NODE_SLOTS.pop(0).shutdown()
            RUNNING_CAMERAS.pop(0)
        else:
            NODE_SLOTS.pop(RUNNING_CAMERAS.index(preferred_slot)).shutdown()
            RUNNING_CAMERAS.remove(preferred_slot)
        rospy.sleep(2)
    NODE_SLOTS.append(RunningImage(camera_url, camera_name))
    RUNNING_CAMERAS.append(camera_name)


def stop_camera(camera_name):
    global RUNNING_CAMERAS, NODE_SLOTS
    if camera_name not in RUNNING_CAMERAS:
        rospy.logwarn("Can't stop not running camera {}".format(camera_name))
        return False
    else:
        NODE_SLOTS.pop(RUNNING_CAMERAS.index(camera_name)).shutdown()
        RUNNING_CAMERAS.remove(camera_name)
        return True


def stop_all():
    for i in NODE_SLOTS:
        i.shutdown()
