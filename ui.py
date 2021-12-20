import serial
import numpy as np
import os
import time
from datetime import datetime
from scipy.io import loadmat

from PyQt5 import uic
from PyQt5.QtCore import QTimer
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
# ARDUINO_PORT = '/dev/tty.usbserial-AC00921Z' # macOS - Robin's MBP
ARDUINO_PORT = 'COM8' # Windows - Lab Microscope PC
ARDUINO_BAUD = 115200

#=========================================================
# Magnet Calibration Info
#=========================================================
Z_OFFSET = 35.2 # z-offset in mm, from height of magnet centre to height of sample
if Z_OFFSET == 0:
    MAGNET_CALIB_MATFILE = 'two_mag_constangle_field_17.27Am2.mat'
else:
    MAGNET_CALIB_MATFILE = f'two_mag_3D_17.27Am2_{Z_OFFSET}mmZoffset.mat'


#=========================================================
# Stepper and Motor Info
#=========================================================
MICROSTEP_FACTOR = 8.0
MOTOR_STEPS_PER_REV = 200.0
STEPS_PER_REV = MICROSTEP_FACTOR * MOTOR_STEPS_PER_REV
MM_PER_REV = 8.0

SMALL_MANUAL_STEPS_LINEAR = (0.25 / MM_PER_REV) * STEPS_PER_REV
SMALL_MANUAL_STEPS_ANGULAR = (0.5 / 360) * STEPS_PER_REV
BIG_MANUAL_STEPS_LINEAR = (5 / MM_PER_REV) * STEPS_PER_REV
BIG_MANUAL_STEPS_ANGULAR = (5 / 360) * STEPS_PER_REV

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
        self.initTime = time.time() # Helpful in some cases to have a global reference time

        # Status flags
        self.isManualControl = False
        self.isAborted = True
        self.isDataRecording = False

        # UI connections - main buttons
        self.button_resetField.clicked.connect(self.resetField)
        self.button_abort.clicked.connect(self.abortMotion)
        self.button_zeroing.clicked.connect(self.beginZeroing)
        self.button_recordData.clicked.connect(self.toggleDataRecording)
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
        # UI connections - check box
        self.checkBox_wigglemode.stateChanged.connect(lambda: self.wiggleToggle(self.checkBox_wigglemode))

        # Create serial connection
        self.ser = serial.Serial(ARDUINO_PORT, ARDUINO_BAUD, timeout=1)

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
        if Z_OFFSET == 0:
            mat_r = np.squeeze(matfile['r']) * 1000 # Convert m to mm
            mat_B = np.squeeze(matfile['B_map_norm']) * 1000 # Convert T to mT
            self.mat_r_rev = np.flip(mat_r) # np.interp requires increasing xp array as per documentation, this is for that case
            self.mat_B_rev = np.flip(mat_B) # np.interp requires increasing xp array as per documentation, this is for that case
        else:
            mat_map = np.squeeze(matfile['mag_angles_and_positions'])

        self.new_msg = ''
        self.msg_history = ['']

    def abortMotion(self):
        # send stop signal
        self.sendStop()

        # Turn off wiggle
        self.checkBox_wigglemode.setChecked(False)

        # lock out controls,
        # prevent sending further field values
        self.isAborted = True

    def beginZeroing(self):
        # User must manually navigate to the zeroing point and then hit the zeroing button

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

    def toggleDataRecording(self):
        '''
        sets/unsets data recording flag, so that self.update() can know if it's supposed to be recording data or not
        '''
        datetime_string = datetime.today().strftime('%Y-%m-%d_%H-%M-%S')
        time_string = datetime.today().strftime('%H:%M:%S.%f')
        maxmsglines = 2

        if not self.isDataRecording:
            self.fout = open(f'doublemagdata_{datetime_string}.txt', 'a')
            self.fout.write(f'started at {time_string}\n')

            headings = [
                'time (s)',
                'commanded field intensity (mT)',
                'commanded field angle (deg)',
                'motor A position (mm)',
                'motor B position (mm)',
                'motor C position (mm)',
                'motor D position (mm)',
                'limit switch 1 status',
                'limit switch 2 status'
            ]
            self.fout.write(','.join(headings) + '\n')

            # Set data start time so recording loop can record the time column appropriately
            self.dataRecordingStarttime = time.time()

            # Update the message box
            new_msg = f'data recording started at: {datetime_string}'
            txt = self.output_dataRecordingStatus.toPlainText().split('\n')
            txt.append(new_msg)
            if len(txt) > maxmsglines:
                txt.pop(0)
            self.output_dataRecordingStatus.setText('\n'.join(txt))

            # Flag to start recording data
            self.isDataRecording = True
        else:
            # Flag to stop recording data
            self.isDataRecording = False

            # Update the message box
            new_msg = f'{time_string}: stopped recording'
            txt = self.output_dataRecordingStatus.toPlainText().split('\n')
            txt.append(new_msg)
            if len(txt) > maxmsglines:
                txt.pop(0)
            self.output_dataRecordingStatus.setText('\n'.join(txt))

            # Close file
            self.fout.close()

    def motorA_l(self):
        self.isManualControl = True
        self.sendSteps(motor=0, abs_steps=self.motor_step[0] + SMALL_MANUAL_STEPS_LINEAR)
    def motorA_r(self):
        self.isManualControl = True
        self.sendSteps(motor=0, abs_steps=self.motor_step[0] - SMALL_MANUAL_STEPS_LINEAR)
    def motorB_u(self):
        self.isManualControl = True
        self.sendSteps(motor=1, abs_steps=self.motor_step[1] - SMALL_MANUAL_STEPS_ANGULAR)
    def motorB_d(self):
        self.isManualControl = True
        self.sendSteps(motor=1, abs_steps=self.motor_step[1] + SMALL_MANUAL_STEPS_ANGULAR)
    def motorC_u(self):
        self.isManualControl = True
        self.sendSteps(motor=2, abs_steps=self.motor_step[2] + SMALL_MANUAL_STEPS_ANGULAR)
    def motorC_d(self):
        self.isManualControl = True
        self.sendSteps(motor=2, abs_steps=self.motor_step[2] - SMALL_MANUAL_STEPS_ANGULAR)
    def motorD_l(self):
        self.isManualControl = True
        self.sendSteps(motor=3, abs_steps=self.motor_step[3] - SMALL_MANUAL_STEPS_LINEAR)
    def motorD_r(self):
        self.isManualControl = True
        self.sendSteps(motor=3, abs_steps=self.motor_step[3] + SMALL_MANUAL_STEPS_LINEAR)
    def motorA_l_2(self):
        self.isManualControl = True
        self.sendSteps(motor=0, abs_steps=self.motor_step[0] + BIG_MANUAL_STEPS_LINEAR)
    def motorA_r_2(self):
        self.isManualControl = True
        self.sendSteps(motor=0, abs_steps=self.motor_step[0] - BIG_MANUAL_STEPS_LINEAR)
    def motorB_u_2(self):
        self.isManualControl = True
        self.sendSteps(motor=1, abs_steps=self.motor_step[1] - BIG_MANUAL_STEPS_ANGULAR)
    def motorB_d_2(self):
        self.isManualControl = True
        self.sendSteps(motor=1, abs_steps=self.motor_step[1] + BIG_MANUAL_STEPS_ANGULAR)
    def motorC_u_2(self):
        self.isManualControl = True
        self.sendSteps(motor=2, abs_steps=self.motor_step[2] + BIG_MANUAL_STEPS_ANGULAR)
    def motorC_d_2(self):
        self.isManualControl = True
        self.sendSteps(motor=2, abs_steps=self.motor_step[2] - BIG_MANUAL_STEPS_ANGULAR)
    def motorD_l_2(self):
        self.isManualControl = True
        self.sendSteps(motor=3, abs_steps=self.motor_step[3] - BIG_MANUAL_STEPS_LINEAR)
    def motorD_r_2(self):
        self.isManualControl = True
        self.sendSteps(motor=3, abs_steps=self.motor_step[3] + BIG_MANUAL_STEPS_LINEAR)

    def resetField(self):
        self.slider_fieldAngle.setValue(0.0)
        self.slider_fieldIntensity.setValue(1.0)
        # reset button also resets the control type.
        self.isAborted = False
        self.isManualControl = False

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

        print(f'field intensity: {field}, angle: {angle}')

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

    def sendField_zoffset(self, field, angle):
        '''
        Send desired field to arduino, for arduino to control. Considers a non-zero z-offset.

            Parameters:
                field (float): desired magnetic flux intensity (mT)
                angle (float): desired field angle (deg)
            Returns:
                None

        Replaces send_msg()
        '''

        print(f'field intensity: {field}, angle: {angle}')

        if field > 25 or field < 1:
            print('field out of bounds: {field} mT')
        else:
            desired_r = np.interp(field, self.mat_map[round(field-1)][0], self.mat_map[round(field-1)][2])
            print(f'desired magnet sep: {desired_r}')
            desired_ang = np.interp(field, self.mat_map[round(field-1)][0], self.mat_map[round(field-1)][1])
            print(f'desired magnet angle: {desired_ang}')

            # Absolute step target - linear axis
            step_a = self.motor_step_0[0] + self.__distance_to_steps(desired_r - self.motor_pos_0[0])
            step_d = self.motor_step_0[3] + self.__distance_to_steps(desired_r - self.motor_pos_0[3])

            # Absolute step target - rotational axis
            step_b = self.motor_step_0[1] + self.__angle_to_steps(desired_ang - self.motor_pos_0[1])
            step_c = self.motor_step_0[2] - self.__angle_to_steps(desired_ang - self.motor_pos_0[2]) # Subtract because motor c local axis is opposite of global axis

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
        print(f'sent message: {msg}')

    def setupTimer(self):
        self.timer = QTimer()
        self.timer.timeout.connect(self.update)
        self.timer.start(1000.0/12.0) # milliseconds between updates

    def update(self):
        # Get latest readings
        self.updateReadings()

        # Send field commands
        if not self.isAborted and not self.isManualControl:
            if Z_OFFSET == 0:
                self.sendField(field=self.slider_fieldIntensity.value(), angle=self.slider_fieldAngle.value())
            else:
                self.sendField_zoffset(field=self.slider_fieldIntensity.value(), angle=self.slider_fieldAngle.value())

        # Update UI
        self.updateUI()

        # Record data
        try:
            self.fout
            self.dataRecordingStarttime
        except AttributeError:
            pass
        else:
            if self.isDataRecording:
                dataTime = time.time() - self.dataRecordingStarttime
                self.fout.write(
                    str(round(dataTime, 3)) +
                    ',' +
                    str(self.slider_fieldIntensity.value()) +
                    ',' +
                    str(self.slider_fieldAngle.value()) +
                    ',' +
                    ','.join([str(i) for i in self.motor_pos]) +
                    ',' +
                    ','.join([str(i) for i in self.limitSwitchStatus]) +
                    '\n')

    def updateReadings(self):
        try:
            self.new_msg = self.ser.readline().decode().rstrip()
            incoming_serial_words = self.new_msg.split(',')
            # Check the first entry in values to see what kind of data it is
            if incoming_serial_words[0] == 'data':
                # Assumed message contents:
                # 0: 'data'
                # 1-4: step value of motor A, B, C, D
                # 5: 1 or 0 representing limit switch 1
                # 6: 1 or 0 representing limit switch 2
                try:
                    # Update position
                    self.motor_step[:] = np.array([float(x) for x in incoming_serial_words[1:5]])
                    stepdiff = self.motor_step - self.motor_step_0
                    self.motor_pos[0] = self.motor_pos_0[0] + self.__steps_to_distance(stepdiff[0])
                    self.motor_pos[1] = self.motor_pos_0[1] + self.__steps_to_angle(stepdiff[1])
                    self.motor_pos[2] = -(self.motor_pos_0[2] + self.__steps_to_angle(stepdiff[2]))
                    self.motor_pos[3] = self.motor_pos_0[3] + self.__steps_to_distance(stepdiff[3])

                    # Update limit switch
                    self.limitSwitchStatus = [int(x) for x in incoming_serial_words[5:]]
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

    def wiggleSignal(self):
        period = 10 #seconds

        angle = 90 + 85*np.sin(2*np.pi*(time.time()-self.initTime)/period)
        self.slider_fieldAngle.setValue(angle)

    def wiggleToggle(self, checkbox):
        commandTimeResolution = 1000 # milliseconds between timer activations - does not reflect period but rather the command time resolution

        if checkbox.isChecked():
            # Start timer
            try:
                self.wiggleTimer
            except AttributeError:
                # Timer hasn't been created for the first time, create it
                self.wiggleTimer = QTimer()
                self.wiggleTimer.timeout.connect(self.wiggleSignal)
            self.wiggleTimer.start(commandTimeResolution)
        else:
            # Stop timer
            try:
                self.wiggleTimer
            except AttributeError:
                pass
            else:
                self.wiggleTimer.stop()

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
