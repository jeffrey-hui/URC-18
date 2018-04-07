import math

debug = False  # seriously pycharm?

if debug:
    from PySide.QtGui import QWidget, QPaintEvent, QPainter, QPen, QHBoxLayout, QVBoxLayout, QSizePolicy, QBrush
    from PySide.QtCore import Slot, Qt, QSize, Signal

    # above gives autocomplete
else:
    from python_qt_binding.QtGui import QPaintEvent, QPen, QPainter, QBrush
    from python_qt_binding.QtCore import Slot, Qt, QSize, Signal
    from python_qt_binding.QtWidgets import QWidget, QHBoxLayout, QVBoxLayout, QSizePolicy


class HorizonWidget(QWidget):
    def __init__(self, *args, **kwargs):
        super(HorizonWidget, self).__init__(*args, **kwargs)

        self.roll = 0
        self.pitch = 0

    def hasHeightForWidth(self):
        return True

    def heightForWidth(self, w):
        return w

    def sizeHint(self, *args, **kwargs):
        return QSize(250, 250)

    def minimumSizeHint(self, *args, **kwargs):
        return QSize(150, 150)

    def paintEvent(self, *args, **kwargs):
        painter = QPainter(self)
        painter.setRenderHint(painter.Antialiasing)

        width = self.width()
        height = self.height()

        pen = QPen()

        deg1 = -math.degrees(self.pitch)

        painter.setPen(pen)
        painter.setBrush(Qt.blue)

        if deg1 > 80:
            deg1 = 80
        elif deg1 < -80:
            deg1 = -80

        painter.drawChord(0, 0, width, height, (0+deg1)*16, (180-deg1*2)*16)

        painter.setBrush(Qt.green)

        painter.drawChord(0, 0, width, height, (0-deg1)*16, -(180-deg1*2)*16)

        pen = QPen()
        pen.setBrush(Qt.black)

        pen.setWidth(2)
        painter.setBrush(QBrush())
        painter.setPen(pen)
        painter.drawEllipse(0, 0, width, height)

    def set_pitch_roll(self, pitch, roll):
        self.pitch = pitch
        self.roll = roll
        self.update()