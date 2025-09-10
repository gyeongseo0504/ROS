from PyQt5.QtWidgets import *
import sys

class Window(QWidget):
    def __init__(self):
        QWidget.__init__(self)
        layout = QGridLayout()
        self.setLayout(layout)
        self.dial = QDial()
        self.dial.setMinimum(0)
        self.dial.setMaximum(100)
        self.dial.setValue(40)
        self.dial.valueChanged.connect(self.sliderMoved)
        layout.addWidget(self.dial)

    def sliderMoved(self):
        print("Dial value = %i" % (self.dial.value()))

app = QApplication(sys.argv)
screen = Window()
screen.show()
sys.exit(app.exec_())
