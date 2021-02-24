import time
import serial
import numpy as np
from PyQt5.QtCore import pyqtSignal, QObject, QThread

class Thread_Wiggler(QObject):
    angleSignal = pyqtSignal(float)

    def __init__(self, ser):
        super().__init__()
        self.ser = ser # serial

    def run(self):
        startTime = time.time()
        while True:
