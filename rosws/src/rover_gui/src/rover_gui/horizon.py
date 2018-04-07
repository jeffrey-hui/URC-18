from qt_gui.plugin import Plugin
import PyKDL
import rospy
import sensor_msgs.msg

from horizon_widget import HorizonWidget, QSizePolicy, QWidget, QHBoxLayout, Signal, Slot


class HorizonPlugin(Plugin):
    emit_data = Signal(float, float)

    def __init__(self, context):
        print("okeydoket")
        super(HorizonPlugin, self).__init__(context)

        self.horizon = HorizonWidget()

        self.horizon.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Fixed)

        self.widget = QWidget()
        self.layout = QHBoxLayout()

        self.layout.addWidget(self.horizon)

        self.widget.setLayout(self.layout)
        self.widget.setWindowTitle("Horizon")
        self.widget.setObjectName("hor_wid")
        self.sub = rospy.Subscriber("/imu/data", sensor_msgs.msg.Imu, callback=self.on_new_imu_data, queue_size=5)
        self.emit_data.connect(self.set_new_data)

        context.add_widget(self.widget)

    def on_new_imu_data(self, msg):
        pass

    @Slot(float, float)
    def set_new_data(self, pitch, roll):
        pass
