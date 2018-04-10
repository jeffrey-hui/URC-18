import math
from qt_gui.plugin import Plugin
import rospy
import std_srvs.srv

debug = True  # seriously pycharm?

if debug:
    from PySide.QtGui import QWidget, QVBoxLayout, QSizePolicy, QPushButton, QLineEdit
    from PySide.QtCore import Slot, Qt, QSize, Signal
else:
    from python_qt_binding.QtGui import QPaintEvent, QPen, QPainter, QBrush
    from python_qt_binding.QtCore import Slot, Qt, QSize, Signal
    from python_qt_binding.QtWidgets import QWidget, QHBoxLayout, QVBoxLayout, QSizePolicy, QLineEdit


class ToggleWidget(QWidget):
    def __init__(self, *args, **kwargs):
        super(ToggleWidget, self).__init__(*args, **kwargs)

        self.line_edit = QLineEdit(self)
        self.button = QPushButton("Toggle teleoperation", self)

        self.layout = QVBoxLayout()
        self.layout.addWidget(self.line_edit)
        self.layout.addStretch()
        self.layout.addWidget(self.button)
        self.layout.addStretch()


class TogglePlugin(Plugin):
    def __init__(self, context):
        super(TogglePlugin, self).__init__(context)

        self.widget = ToggleWidget()