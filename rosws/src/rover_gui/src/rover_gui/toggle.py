import math
from qt_gui.plugin import Plugin
import rospy
import std_srvs.srv

debug = False# seriously pycharm?

if debug:
    from PySide.QtGui import QWidget, QVBoxLayout, QSizePolicy, QPushButton, QLineEdit, QLabel, QInputDialog
    from PySide.QtCore import Slot, Qt, QSize, Signal
else:
    from python_qt_binding.QtCore import Slot, Qt, QSize, Signal
    from python_qt_binding.QtWidgets import QWidget, QHBoxLayout, QVBoxLayout, QLineEdit, QPushButton, QLabel, QInputDialog


class ToggleWidget(QWidget):
    toggled = Signal()
    service_changed = Signal()

    def __init__(self, *args, **kwargs):
        super(ToggleWidget, self).__init__(*args, **kwargs)

        self.line_edit = QLineEdit(self)
        self.button = QPushButton("Toggle", self)
        self.label = QLabel()

        self.layout = QVBoxLayout()
        self.layout.addWidget(self.line_edit)
        self.layout.addStretch()
        self.layout.addWidget(self.button)
        self.layout.addWidget(self.label)
        self.layout.addStretch()
        self.button.clicked.connect(self.on_click)
        self.line_edit.editingFinished.connect(self.on_edit)

        self.setLayout(self.layout)
        self.s = False

    def set_label(self, k):
        self.button.setText(k)

    def update_me(self):
        self.label.setText("Current state: {}".format("ON" if self.s else "OFF"))

    def service(self):
        return str(self.line_edit.text())

    def set_service(self, s):
        self.line_edit.setText(s)

    def set_state(self, s):
        self.s = s
        self.update_me()

    @Slot()
    def on_edit(self):
        self.service_changed.emit()

    @Slot()
    def on_click(self):
        self.s = not self.s
        self.update_me()
        self.toggled.emit()


class TogglePlugin(Plugin):
    def __init__(self, context):
        super(TogglePlugin, self).__init__(context)

        self.widget = ToggleWidget()

        self.widget.toggled.connect(self.on_toggled)
        self.widget.setWindowTitle("Toggle")
        self.widget.service_changed.connect(self.on_service_change)

        self.service = None
        self.label = "Toggle"

        context.add_widget(self.widget)

    def shutdown_plugin(self):
        if self.service is not None:
            self.service.close()

    def save_settings(self, plugin_settings, instance_settings):
        instance_settings.set_value("service", self.widget.service())
        instance_settings.set_value("label", self.label)

    def restore_settings(self, plugin_settings, instance_settings):
        self.widget.set_service(instance_settings.value("service"))
        self.label = instance_settings.value("label", default_value=self.label)
        self.widget.set_label(self.label)
        self.on_service_change()

    @Slot()
    def on_service_change(self):
        if self.service is not None:
            self.service.close()
        try:
            self.service = rospy.ServiceProxy(self.widget.service(), std_srvs.srv.SetBool)
            self.widget.set_state(self.service(std_srvs.srv.SetBoolRequest(False)).success)
        except rospy.ServiceException:
            self.service = None

    @Slot()
    def on_toggled(self):
        if self.service is not None:
            self.service(std_srvs.srv.SetBoolRequest(True))

    def trigger_configuration(self):
        label, a = QInputDialog.getText(self.widget, "Set label text", "Enter label text for button", QLineEdit.Normal, self.label)
        if a:
            self.label = label
            self.widget.set_label(self.label)
