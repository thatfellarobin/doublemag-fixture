import sys
from PyQt5.QtWidgets import QApplication

from ui import DoubleMagnetGUI

if __name__ == '__main__':
    app = QtGui.QApplication(sys.argv)
    window = DoubleMagnetGUI()
    window.show()
    sys.exit(app.exec_())
