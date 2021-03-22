import serial
import numpy as np
import os
import time
from scipy.io import loadmat

from PyQt5 import uic
from PyQt5.QtCore import QTimer, QThread
from PyQt5.QtWidgets import QMainWindow

# from subthreads import Thread_Wiggler

#=========================================================
# UI Setup
#=========================================================
qtCreatorFile = 'doublemag.ui'
Ui_MainWindow, QtBaseClass = uic.loadUiType(qtCreatorFile)
MSG_HISTORY_LENGTH = 3

#=========================================================
# Serial Info
#=========================================================
ARDUINO_PORT = '/dev/tty.usbserial-AC00921Z'
ARDUINO_BAUD = 9600

#=========================================================
# Magnet Calibration Info
#=========================================================
MAGNET_CALIB_MATFILE ='two_mag_constangle_field_17.27Am2.mat'

#=========================================================
# Stepper and Motor Info
#=========================================================
MICROSTEP_FACTOR = 8.0
MOTOR_STEPS_PER_REV = 200.0
STEPS_PER_REV = MICROSTEP_FACTOR * MOTOR_STEPS_PER_REV
MM_PER_REV = 8.0

MANUAL_STEPS_LINEAR = (5 / MM_PER_REV) * STEPS_PER_REV
MANUAL_STEPS_ANGULAR = (5 / 360) * STEPS_PER_REV
BIG_MANUAL_STEPS_LINEAR = (0.25 / MM_PER_REV) * STEPS_PER_REV
BIG_MANUAL_STEPS_ANGULAR = (0.5 / 360) * STEPS_PER_REV

class DoubleMagnetGUI(QMainWindow, Ui_MainWindow):
    # Notes on direction
    # - Positive # of steps rotates the motor shaft clockwise as viewed from the motor body.
    # - For the linear axis, clockwise rotation moves the magnet away from the workspace

    # Notes on units
    # - fixture records position in mm
    def __init__(self):
        QMainWindow.__init__(self)
        Ui_MainWindow.__init__(self)
        self.setupUi(self)
        self.setupTimer()

        # Status flags
        self.isManualControl = False
        self.isAltControl = False # for subthreads etc. TODO:
        self.isAborted = True

        # UI connections - main buttons
        self.button_resetField.clicked.connect(self.resetField)
        self.button_abort.clicked.connect(self.abortMotion)
        self.button_zeroing.clicked.connect(self.beginZeroing)
        # UI connections - manual controls
        self.button_motorA_l.clicked.connect(self.motorA_l)
        self.button_motorA_r.clicked.connect(self.motorA_r)
        self.button_motorB_u.clicked.connect(self.motorB_u)
        self.button_motorB_d.clicked.connect(self.motorB_d)
        self.button_motorC_u.clicked.connect(self.motorC_u)
        self.button_motorC_d.clicked.connect(self.motorC_d)
        self.button_motorD_l.clicked.connect(self.motorD_l)
        self.button_motorD_r.clicked.connect(self.motorD_r)
        self.button_motorA_l_2.clicked.connect(self.motorA_l_2)
        self.button_motorA_r_2.clicked.connect(self.motorA_r_2)
        self.button_motorB_u_2.clicked.connect(self.motorB_u_2)
        self.button_motorB_d_2.clicked.connect(self.motorB_d_2)
        self.button_motorC_u_2.clicked.connect(self.motorC_u_2)
        self.button_motorC_d_2.clicked.connect(self.motorC_d_2)
        self.button_motorD_l_2.clicked.connect(self.motorD_l_2)
        self.button_motorD_r_2.clicked.connect(self.motorD_r_2)

        # Create serial connection
        self.ser = serial.Serial(ARDUINO_PORT, ARDUINO_BAUD, timeout=1)

        # Setup subthreads
        # self.thread = QThread()
        # self.wiggle_thread = Thread_Wiggler(self.ser)
        # self.wiggle_thread.moveToThread(self.thread)
        # self.thread.started.connect(self.wiggle_thread.run)
        # self.wiggle_thread.angleSignal.connect(TODO:)

        # Variables to track motor position
        self.motor_step_0 = np.array([0., 0., 0., 0.])
        self.motor_step = np.array([0., 0., 0., 0.])
        self.motor_pos_0 = np.array([0., 0., 0., 0.])
        self.motor_pos = np.array([0., 0., 0., 0.])
        self.limitSwitchStatus = [0, 0]

        # Variables to track magnetic reading
        self.mag = np.array([0., 0.]) # magnitude (mT) and angle (deg)

        # Load calibration matfile
        matfile = loadmat(MAGNET_CALIB_MATFILE)
        mat_r = np.squeeze(matfile['r']) * 1000 # Convert m to mm
        mat_B = np.squeeze(matfile['B_map_norm']) * 1000 # Convert T to mT
        self.mat_r_rev = np.flip(mat_r) # np.interp requires increasing xp array as per documentation, this is for that case
        self.mat_B_rev = np.flip(mat_B) # np.interp requires increasing xp array as per documentation, this is for that case

        self.new_msg = ''
        self.msg_history = ['']

    def abortMotion(self):
        # send stop signal
        self.sendStop()

        # lock out controls,
        # prevent sending further field values
        self.isAborted = True

    def beginZeroing(self):
        # Manually navigate to the zeroing point and then hit the zeroing button
        zeroing_point=195
        zeroing_separation=45.75

        # Zero the linear axis
        self.motor_pos_0[0] = zeroing_point - zeroing_separation
        self.motor_pos_0[3] = zeroing_point - zeroing_separation

        # Zero the rotary axis
        self.motor_pos_0[1] = 0
        self.motor_pos_0[2] = 0


        # Match zeroed position with zeroed steps.
        for i in range(4):
            self.motor_step_0[i] = self.motor_step[i]

        print(zeroing_point)
        print(zeroing_separation)
        print(self.motor_pos_0)
        print(self.motor_step_0)


    def setupTimer(self):
        self.timer = QTimer()
        self.timer.timeout.connect(self.update)
        self.timer.start(1000.0/12.0) # milliseconds between updates

    def update(self):
        # Get latest readings
        self.updateReadings()

        # Send field commands
        if not self.isAborted and not self.isManualControl and not self.isAltControl:
            self.sendField(field=self.slider_fieldIntensity.value(), angle=self.slider_fieldAngle.value())

        # Update UI
        self.updateUI()
        # raise NotImplementedError

    def updateReadings(self):
        try:
            self.new_msg = self.ser.readline().decode().rstrip()
            incoming_serial_words = self.new_msg.split(',')
            # Check the first entry in values to see what kind of data it is
            if incoming_serial_words[0] == 'data':
                # Assumed message contents:
                # 0: 'data'
                # 1-4: step value of motor A, B, C, D
                # 5: magnetic field magnitude (mT)
                # 6: magnetic field angle (deg)
                # 7: 1 or 0 representing limit switch 1
                # 8: 1 or 0 representing limit switch 2
                try:
                    # Update position
                    self.motor_step[:] = np.array([float(x) for x in incoming_serial_words[1:5]])
                    stepdiff = self.motor_step - self.motor_step_0
                    self.motor_pos[0] = self.motor_pos_0[0] + self.__steps_to_distance(stepdiff[0])
                    self.motor_pos[1] = self.motor_pos_0[1] + self.__steps_to_angle(stepdiff[1])
                    self.motor_pos[2] = -(self.motor_pos_0[2] + self.__steps_to_angle(stepdiff[2]))
                    self.motor_pos[3] = self.motor_pos_0[3] + self.__steps_to_distance(stepdiff[3])

                    # Update magnetic
                    # self.mag[0] = incoming_serial_words[5]
                    # self.mag[1] = incoming_serial_words[6]

                    # Update limit switch
                    self.limitSwitchStatus = [int(x) for x in incoming_serial_words[7:]]
                except ValueError:
                    pass
                except IndexError:
                    pass
        except:
            pass

    def updateUI(self):
        # Update the commanded field readout according to slider position
        self.input_fieldAngle.setText(str(self.slider_fieldAngle.value()))
        self.input_fieldIntensity.setText(str(self.slider_fieldIntensity.value()))

        # Update the received messages box with latest messages if the new message is unique
        if self.new_msg != self.msg_history[-1]:
            self.msg_history.append(self.new_msg)
            if len(self.msg_history) > MSG_HISTORY_LENGTH:
                self.msg_history.pop(0)
        self.output_latestMsg.setPlainText('\n'.join(self.msg_history))

        # Update the field readings from the latest message
        # self.output_fieldIntensity.setText(str(self.mag[0]))
        # self.output_fieldAngle.setText(str(self.mag[1]))

    def resetField(self):
        self.slider_fieldAngle.setValue(0.0)
        self.slider_fieldIntensity.setValue(1.0)
        # reset button also resets the control type.
        self.isAborted = False
        self.isManualControl = False
        self.isAltControl = False

    def sendField(self, field, angle):
        '''
        Send desired field to arduino, for arduino to control.

            Parameters:
                field (float): desired magnetic flux intensity (mT)
                angle (float): desired field angle (deg)
            Returns:
                None

        Replaces send_msg()
        '''

        print(f'field: {field}')
        print(f'angle: {angle}')

        if field > 35 or field < 1:
            print('field out of bounds: {field} mT')
        else:
            desired_r = np.interp(field, self.mat_B_rev, self.mat_r_rev) # use reversed because xp must be increasing as per np.interp documetation
            print(f'desired r: {desired_r}')

            # Absolute step target - linear axis
            step_a = self.motor_step_0[0] + self.__distance_to_steps(desired_r - self.motor_pos_0[0])
            step_d = self.motor_step_0[3] + self.__distance_to_steps(desired_r - self.motor_pos_0[3])

            # Absolute step target - rotational axis
            step_b = self.motor_step_0[1] + self.__angle_to_steps(angle - self.motor_pos_0[1])
            step_c = self.motor_step_0[2] - self.__angle_to_steps(angle - self.motor_pos_0[2]) # Subtract because motor c local axis is opposite of global axis

            self.sendSteps(motor=0, abs_steps=step_a)
            self.sendSteps(motor=1, abs_steps=step_b)
            self.sendSteps(motor=2, abs_steps=step_c)
            self.sendSteps(motor=3, abs_steps=step_d)

    def sendSteps(self, motor, abs_steps=0):
        '''
        Send movement command to a single motor
            Parameters:
                motor (int): 0-3 indicating which motor is selected
                abs_steps (int): the step value target that the motor should move towards
            Returns:
                None
        '''

        if abs_steps > 99999 or abs_steps < -99999:
            raise ValueError("# steps must be between -999999 and 999999")
        if motor > 3 or motor < 0:
            raise ValueError("Motor must be integer from 0-3")

        msg = 'm' + str(motor) + str(abs(int(abs_steps))).zfill(5) + ('-' if abs_steps < 0 else '+') + '\n'
        msg_encode = msg.encode(encoding='ascii')
        self.ser.write(msg_encode)
        print(f'sent message: {msg}')

    def sendStop(self):
        msg = 's\n'.encode(encoding='ascii')
        self.ser.write(msg)

    # region Motor manual commands
    def motorA_l(self):
        self.isManualControl = True
        self.sendSteps(motor=0, abs_steps=self.motor_step[0] + MANUAL_STEPS_LINEAR)
    def motorA_r(self):
        self.isManualControl = True
        self.sendSteps(motor=0, abs_steps=self.motor_step[0] - MANUAL_STEPS_LINEAR)
    def motorB_u(self):
        self.isManualControl = True
        self.sendSteps(motor=1, abs_steps=self.motor_step[1] - MANUAL_STEPS_ANGULAR)
    def motorB_d(self):
        self.isManualControl = True
        self.sendSteps(motor=1, abs_steps=self.motor_step[1] + MANUAL_STEPS_ANGULAR)
    def motorC_u(self):
        self.isManualControl = True
        self.sendSteps(motor=2, abs_steps=self.motor_step[2] + MANUAL_STEPS_ANGULAR)
    def motorC_d(self):
        self.isManualControl = True
        self.sendSteps(motor=2, abs_steps=self.motor_step[2] - MANUAL_STEPS_ANGULAR)
    def motorD_l(self):
        self.isManualControl = True
        self.sendSteps(motor=3, abs_steps=self.motor_step[3] - MANUAL_STEPS_LINEAR)
    def motorD_r(self):
        self.isManualControl = True
        self.sendSteps(motor=3, abs_steps=self.motor_step[3] + MANUAL_STEPS_LINEAR)
    def motorA_l_2(self):
        self.isManualControl = True
        self.sendSteps(motor=0, abs_steps=self.motor_step[0] + MANUAL_STEPS_LINEAR)
    def motorA_r_2(self):
        self.isManualControl = True
        self.sendSteps(motor=0, abs_steps=self.motor_step[0] - MANUAL_STEPS_LINEAR)
    def motorB_u_2(self):
        self.isManualControl = True
        self.sendSteps(motor=1, abs_steps=self.motor_step[1] - MANUAL_STEPS_ANGULAR)
    def motorB_d_2(self):
        self.isManualControl = True
        self.sendSteps(motor=1, abs_steps=self.motor_step[1] + MANUAL_STEPS_ANGULAR)
    def motorC_u_2(self):
        self.isManualControl = True
        self.sendSteps(motor=2, abs_steps=self.motor_step[2] + MANUAL_STEPS_ANGULAR)
    def motorC_d_2(self):
        self.isManualControl = True
        self.sendSteps(motor=2, abs_steps=self.motor_step[2] - MANUAL_STEPS_ANGULAR)
    def motorD_l_2(self):
        self.isManualControl = True
        self.sendSteps(motor=3, abs_steps=self.motor_step[3] - MANUAL_STEPS_LINEAR)
    def motorD_r_2(self):
        self.isManualControl = True
        self.sendSteps(motor=3, abs_steps=self.motor_step[3] + MANUAL_STEPS_LINEAR)
    # endregion

    def __steps_to_distance(self, steps):
        '''
        Returns mm displacement for a given number of steps
        '''
        return (steps / STEPS_PER_REV) * MM_PER_REV

    def __distance_to_steps(self, distance):
        '''
        Returns steps for a given mm displacement
        '''
        return (distance /MM_PER_REV) * STEPS_PER_REV

    def __steps_to_angle(self, steps):
        '''
        Returns angle displacement (degrees) for a given number of steps
        '''
        return (steps / STEPS_PER_REV) * 360.0

    def __angle_to_steps(self, angle):
        '''
        Returns steps for a given angle displacement (degrees)
        '''
        return (angle / 360.0) * STEPS_PER_REV
