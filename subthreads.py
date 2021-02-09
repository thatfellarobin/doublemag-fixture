from PyQt5 import QtCore

import time
import serial


class ArduinoManager(QtCore.QThread):
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

    def send_msg(self, motor, step_target):
        '''
        Send messages to the arduino about how the motors should move.
            Parameters:
                motor (char): 'a', 'b', 'c', or 'd' indicating which motor is selected
                step_target (int): the step value target that the motor should move towards
            Returns:
                None
        '''

        if steps > 999999 or steps < -999999:
            raise ValueError("# steps must be between -999999 and 999999")

        if steps >= 0:
            direction = '+'
        else:
            direction = '-'

        msg = motor + direction + '\n'
        msg_encode = msg.encode(encoding='ascii')
        self.ser.write(msg_encode)
