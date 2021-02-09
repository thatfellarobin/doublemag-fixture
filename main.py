import sys
from PyQt5 import QtCore, QtGui, uic

from controllers import DoubleMagnetGUI

if __name__ == '__main__':
    app = QtGui.QApplication(sys.argv)
    window = DoubleMagnetGUI()
    window.show()
    sys.exit(app.exec_())
