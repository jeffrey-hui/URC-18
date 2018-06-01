#!/usr/bin/env python
import rospy
import rover_cameras.srv
import rover_cameras.msg
from prompt_toolkit import prompt
from prompt_toolkit.completion import Completer, Completion
from prompt_toolkit.history import History, InMemoryHistory

lazy_cams = []
history = InMemoryHistory()

class SwapperCliCompleter(Completer):
    COMMANDS = ["quit", "load", "unload", "list", "?"]

    def __init__(self):
        super(SwapperCliCompleter, self).__init__()

    def complete_word_from_context(self, document, ctx):
        word_before_cursor = document.get_word_before_cursor(WORD=False)
        word_before_cursor = word_before_cursor.lower()

        match = lambda word: word_before_cursor in word
        return [(unicode(x), -len(word_before_cursor)) for x in ctx if match(x)]

    def get_completions(self, document, complete_event):
        l = document.current_line

        current_split_state = l.split(" ")
        cur_word = document.get_word_under_cursor()
        if cur_word not in current_split_state or cur_word == "":
            cur_ind = len(current_split_state)-1
        else:
            cur_ind = current_split_state.index(cur_word)
        if cur_ind < 1:
            for i in self.complete_word_from_context(document, SwapperCliCompleter.COMMANDS):
                yield Completion(i[0], start_position=i[1])
        else:
            cmd = current_split_state[0]
            if cmd in ["quit", "?", "list"]:
                return
            elif cmd == "load":
                if cur_ind == 3:
                    for i in self.complete_word_from_context(document, lazy_cams):
                        yield Completion(i[0], start_position=i[1])
            elif cmd == "unload" and cur_ind == 1:
                for i in self.complete_word_from_context(document, lazy_cams):
                    yield Completion(i[0], start_position=i[1])


rospy.init_node("swapper_cli")

load_camera = rospy.ServiceProxy("/swapper/load_camera", rover_cameras.srv.LoadCamera)
unload_camera = rospy.ServiceProxy("/swapper/unload_camera", rover_cameras.srv.UnloadCamera)

print "Camera MGMT v1"
print "? for help"

while not rospy.is_shutdown():
    lazy_cams = rospy.wait_for_message("/swapper/cameras", rover_cameras.msg.ActiveCameras).cameras
    i = str(prompt(u"> ", completer=SwapperCliCompleter(), history=history))
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
