import rospy
from controller_manager_msgs.srv import SwitchController, ListControllers, LoadController, ListControllersResponse, LoadControllerRequest, SwitchControllerRequest

cont_server = rospy.ServiceProxy("/controller_manager/switch_controller", SwitchController)
cont_list_server = rospy.ServiceProxy("/controller_manager/list_controllers", ListControllers)
cont_load = rospy.ServiceProxy("/controller_manager/load_controller", LoadController)

NEED_CONTROLLERS = {"drill_controller", "arm_manual_controller"}


def ensure_controllers_are_loaded():
    currently_loaded = cont_list_server()  # type: ListControllersResponse
    loaded_controllers = {
        x.name for x in currently_loaded.controller if x.name in NEED_CONTROLLERS
    }
    if loaded_controllers < NEED_CONTROLLERS:  # do we need more controllers loaded?
        need_to_load = NEED_CONTROLLERS - loaded_controllers
        rospy.loginfo("need to load {}".format(", ".join(need_to_load)))
        for i in need_to_load:
            req = LoadControllerRequest(name=i)
            if not cont_load(req).ok:
                rospy.logerr("couldn't start controller {}".format(i))
                exit(1)
    else:
        rospy.loginfo("all controllers loaded")


def set_manual_mode_on_arm():
    cont_server(SwitchControllerRequest(
        start_controllers=[
            "arm_manual_controller",
            "drill_controller"
        ],
        stop_controllers=[

        ],
        strictness=1
    ))
