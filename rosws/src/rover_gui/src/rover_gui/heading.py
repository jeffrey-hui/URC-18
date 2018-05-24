from qt_gui.plugin import Plugin
import PyKDL
import rospy
import sensor_msgs.msg
from heading_widget import HeadingWidget, QWidget, QHBoxLayout, QVBoxLayout, QSizePolicy, Signal, Slot


class HeadingPlugin(Plugin):
    on_new_heading = Signal(float)

    def __init__(self, context):
        super(HeadingPlugin, self).__init__(context)

        self.heading = HeadingWidget()

        self.heading.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Fixed)

        self.widget = QWidget()
        self.layout = QHBoxLayout()

        self.layout.addWidget(self.heading)

        self.widget.setLayout(self.layout)
        self.widget.setWindowTitle("Heading")
        self.widget.setObjectName("head_wid")

        self.sub = rospy.Subscriber("/imu/data", sensor_msgs.msg.Imu, callback=self.on_new_imu, queue_size=5)

        self.on_new_heading.connect(self.on_new_heading_handler)
        context.add_widget(self.widget)

    def shutdown_plugin(self):
        self.sub.unregister()
        del self.sub

    def on_new_imu(self, m):
        orientation = PyKDL.Rotation.Quaternion(
            m.orientation.x, m.orientation.y, m.orientation.z, m.orientation.w
        )
        rpy = orientation.GetRPY()
        self.on_new_heading.emit(rpy[2])

    @Slot(float)
    def on_new_heading_handler(self, h):
        self.heading.set_heading(h)
