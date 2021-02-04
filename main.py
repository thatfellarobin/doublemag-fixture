import sys
from PyQt5 import QtCore, QtGui, uic

from controllers import DoubleMagnetController

if __name__ == '__main__':
    app = QtGui.QApplication(sys.argv)
    window = doubleMagController()
    window.show()
    sys.exit(app.exec_())
