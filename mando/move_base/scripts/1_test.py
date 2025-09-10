from PyQt5 import QtWidgets


def main():
    app = QtWidgets.QApplication([])
    window = QtWidgets.QMainWindow()
    label = QtWidgets.QLabel('\tHello World!')
    window.setCentralWidget(label)
    window.show()
    app.exec_()


if __name__ == '__main__':
    main()
