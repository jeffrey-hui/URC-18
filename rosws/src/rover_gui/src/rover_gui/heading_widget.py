import math

debug = False  # seriously pycharm?

if debug:
    from PySide.QtGui import QWidget, QPaintEvent, QPainter, QPen, QHBoxLayout, QVBoxLayout, QSizePolicy
    from PySide.QtCore import Slot, Qt, QSize, Signal

    # above gives autocomplete
else:
    from python_qt_binding.QtGui import QPaintEvent, QPen, QPainter
    from python_qt_binding.QtCore import Slot, Qt, QSize, Signal
    from python_qt_binding.QtWidgets import QWidget, QHBoxLayout, QVBoxLayout, QSizePolicy


class HeadingWidget(QWidget):
    def __init__(self, *args, **kwargs):
        super(HeadingWidget, self).__init__(*args, **kwargs)

        self.heading = 1.57  # 0 = east, 2*pi-epsilon = almost east

    def hasHeightForWidth(self):
        return True

    def heightForWidth(self, w):
        return w

    def sizeHint(self, *args, **kwargs):
        return QSize(250, 250)

    def minimumSizeHint(self, *args, **kwargs):
        return QSize(150, 150)

    def set_heading(self, h):
        while h < 0:
            h += 2*math.pi
        while h > 2*math.pi:
            h -= 2*math.pi
        self.heading = h - 0.5
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)

        width = self.width()
        height = self.height()

        pen = QPen()
        pen.setBrush(Qt.black)

        pen.setWidth(2)
        painter.setPen(pen)
        painter.setRenderHint(painter.Antialiasing)
        painter.drawEllipse(0, 0, width, height)

        painter.drawLine(width - 20, height // 2, width-1, height // 2)
        painter.drawLine(0, height // 2, 20, height // 2)
        painter.drawLine(width // 2, 0, width // 2, 20)
        painter.drawLine(width // 2, height-20, width // 2, height-1)

        painter.drawText(0, 20, width, height-20, Qt.AlignHCenter | Qt.AlignTop, "N")
        painter.drawText(0, 0, width, height-20, Qt.AlignHCenter | Qt.AlignBottom, "S")
        painter.drawText(25, 0, width-25, height, Qt.AlignVCenter | Qt.AlignLeft, "W")
        painter.drawText(0, 0, width-25, height, Qt.AlignVCenter | Qt.AlignRight, "E")

        pen.setWidth(4)
        pen.setBrush(Qt.blue)
        painter.setPen(pen)

        x = (math.cos(-self.heading) * width // 2) + width // 2
        y = (math.sin(-self.heading) * height // 2) + height // 2

        painter.drawLine(width // 2, height // 2, x, y)
