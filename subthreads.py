from PyQt5 import QtCore

import time
import serial


class Looper(QtCore.QThread):
    def __init__(self, ser):
        QtCore.QThread.__init__(self)

        self.ser = ser

        # Initiate loop
        raise NotImplementedError

    def loop(self):
        raise NotImplementedError

        # Receive latest serial message from the arduino
        # Update the latest position and field readings

        # Send step target to the arduino
