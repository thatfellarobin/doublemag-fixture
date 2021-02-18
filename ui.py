import serial
import numpy as np
import os
import time

from PyQt5 import uic
from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import QMainWindow

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
# Stepper and Motor Info
#=========================================================
MICROSTEP_FACTOR = 8.0
MOTOR_STEPS_PER_REV = 200.0
STEPS_PER_REV = MICROSTEP_FACTOR * MOTOR_STEPS_PER_REV
MM_PER_REV = 8.0


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

        # UI connections
        self.button_resetField.clicked.connect(self.resetField)

        # Create serial connection
        # self.ser = serial.Serial(ARDUINO_PORT, ARDUINO_BAUD, timeout=1)

        # Variables to track motor position
        self.motor_step_0 = np.array([0., 0., 0., 0.])
        self.motor_step = np.array([0., 0., 0., 0.])
        self.motor_pos_0 = np.array([0., 0., 0., 0.])
        self.motor_pos = np.array([0., 0., 0., 0.])

        # Variables to track magnetic reading
        self.mag = np.array([0., 0.]) # magnitude (mT) and angle (deg)

        self.new_msg = ''
        self.msg_history = ['']

    def setupTimer(self):
        self.timer = QTimer()
        self.timer.timeout.connect(self.update)
        self.timer.start(1000.0/15.0) # milliseconds between updates

    def update(self):
        # Get latest readings
        self.updateReadings()

        # Send field commands
        self.sendField(field=self.slider_fieldAngle.value(), angle=self.slider_fieldIntensity.value())

        # Update UI
        self.updateUI()
        # raise NotImplementedError

    def updateReadings(self):
        try:
            self.new_msg = self.ser.readline().decode().rstrip()
            incoming_serial_words = self.new_msg.split(',')
            # Check the first entry in values to see what kind of data it is
            if incoming_serial_words[0] == 'steps':
                # Assumed message contents:
                # 0: 'steps'
                # 1-4: step value of motor A, B, C, D
                # 5: magnetic field magnitude (mT)
                # 6: magnetic field angle (deg)
                try:
                    # Update position
                    self.motor_step[:] = np.array([float(x) for x in incoming_serial_words[1:5]])
                    stepdiff = self.motor_step - self.motor_step_0
                    self.motor_pos[0] = self.motor_pos_0[0] + self.__steps_to_distance(stepdiff[0])
                    self.motor_pos[1] = self.motor_pos_0[1] + self.__steps_to_angle(stepdiff[1])
                    self.motor_pos[2] = -(self.motor_pos_0[2] + self.__steps_to_angle(stepdiff[2]))
                    self.motor_pos[3] = self.motor_pos_0[3] + self.__steps_to_distance(stepdiff[3])

                    # Update magnetic
                    self.mag[0] = incoming_serial_words[5]
                    self.mag[1] = incoming_serial_words[6]
                except ValueError:
                    pass
                except IndexError:
                    pass
            elif incoming_serial_words[0] == 'endstopviolation':
                # the command is asking the motor to move beyond the end stop
                raise NotImplementedError
            elif incoming_serial_words[0] == 'nonvalidmsg':
                # the message sent to the arduino was not valid
                raise NotImplementedError
            else:
                pass
        except:
            pass

    def updateUI(self):
        # Update the commanded field angle according to slider position
        self.input_fieldAngle.setText(str(self.slider_fieldAngle.value()))
        self.input_fieldIntensity.setText(str(self.slider_fieldIntensity.value()))

        # Update the received messages box with latest messages if the new message is unique
        if self.new_msg != self.msg_history[-1]:
            self.msg_history.append(self.new_msg)
            if len(self.msg_history) > MSG_HISTORY_LENGTH:
                self.msg_history.pop(0)
        self.output_latestMsg.setPlainText('\n'.join(self.msg_history))

        # Update the field readings from the latest message
        self.output_fieldIntensity.setText(str(self.mag[0]))
        self.output_fieldAngle.setText(str(self.mag[1]))

    def resetFieldSliders(self):
        self.slider_fieldAngle.setValue(0.0)
        self.slider_fieldIntensity.setValue(1.0)

    def sendField(self, field, angle):
        '''
        Send desired field to arduino, for arduino to control.
        The sent message contains the following:
        - 1 char: 'f' to indicate the message type (field)
        - 3 chars: digits indicating the field: '###' mT (zero padded in front)
        - 5 chars: digits indicating the angle: '###.#' deg (zero padded in front)

            Parameters:
                field (int): desired magnetic flux intensity (mT)
                angle (float): desired field angle (deg)
            Returns:
                None

        Replaces send_msg()
        '''
        if abs(angle) <= 180 and field >= 1 and field <= 35:
            msg = f'f{field:03d}{angle:05.1f}\n' # 3 characters for field, 5 characters for angle (zero-padded)
            msg = msg.encode(encoding='ascii')
            self.ser.write(msg)

    def sendSteps(self, motor, rel_steps=0):
        '''
        TODO: could use this for manual control
            Parameters:
                motor (char): 'a', 'b', 'c', or 'd' indicating which motor is selected
                step_target (int): the step value target that the motor should move towards
            Returns:
                None
        '''
        raise NotImplementedError

        if rel_steps > 999999 or rel_steps < -999999:
            raise ValueError("# steps must be between -999999 and 999999")

        if rel_steps >= 0:
            direction = '+'
        else:
            direction = '-'

        msg = motor + direction + '\n'
        msg_encode = msg.encode(encoding='ascii')
        self.ser.write(msg_encode)


    def __steps_to_distance(self, steps):
        '''
        Returns mm displacement for a given number of steps
        '''
        return (steps / STEPS_PER_REV) * MM_PER_REV

    def __distance_to_steps(self, position):
        '''
        Returns steps for a given mm displacement
        '''
        return (position /MM_PER_REV) * STEPS_PER_REV

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


    # def draw_position(self):
    #     # Draw the indications of each magnet position
    #     # motor_pos is a list containing the real unit positions of the motors.
    #     # m1 and m4 are linear position (mm)
    #     # m2 and m3 are angular position (deg)
    #     # This should be called every time an update from the arduino is available. It's supposed to just update the most recent line with the new position for a live view.

    #     # if the motors are symmetric enough, calculate and display the field
    #     tol = 0.5 # Angular and mm tolerance to define symmetry
    #     if abs(self.latest_motor_pos[0] - self.latest_motor_pos[3]) < tol and abs(self.latest_motor_pos[1] - self.latest_motor_pos[2]) < tol and self.is_matfile and self.is_zeroed:
    #         r = np.mean([self.latest_motor_pos[0], self.latest_motor_pos[3]]) # No unit conversion because matfile is converted to mm/mT at load
    #         B = np.interp(r, self.mat_r, self.mat_B, left=np.nan, right=np.nan) # No unit conversion because matfile is converted to mm/mT at load
    #         if not np.isnan(B):
    #             print('A: {0:5.1f} mm, B: {1:4.1f} deg, C: {2:4.1f} deg, D: {3:5.1f} mm, wp: {4:d}, B: {5:4.1f} mT ###\r'.format(self.latest_motor_pos[0], self.latest_motor_pos[1], self.latest_motor_pos[2], self.latest_motor_pos[3], self.waypoint_counter, B), end="")
    #         else: # r is outside of the range of the matfile
    #             print('A: {0:5.1f} mm, B: {1:4.1f} deg, C: {2:4.1f} deg, D: {3:5.1f} mm, wp: {4:d}, B out of range ###\r'.format(self.latest_motor_pos[0], self.latest_motor_pos[1], self.latest_motor_pos[2], self.latest_motor_pos[3], self.waypoint_counter), end="")
    #     else:
    #         print('A: {0:5.1f} mm, B: {1:4.1f} deg, C: {2:4.1f} deg, D: {3:5.1f} mm, wp: {4:d}, Fixture nonsymmetric or not zeroed ###\r'.format(self.latest_motor_pos[0], self.latest_motor_pos[1], self.latest_motor_pos[2], self.latest_motor_pos[3], self.waypoint_counter), end="")

    # def loop(self):
    #     while not self.exit_flag:
    #         if not self.pause_flag:
    #             time.sleep(0.03)
    #             self.parse_position()
    #             self.draw_position()

    #     # Exited the loop: stop all motors
    #     self.send_msg(mode='M', motor='a', steps=0)
    #     self.send_msg(mode='M', motor='b', steps=0)
    #     self.send_msg(mode='M', motor='c', steps=0)
    #     self.send_msg(mode='M', motor='d', steps=0)

    #     self.flush_input()
    #     time.sleep(0.05)

    #     # Close serial port
    #     self.ser.close()

    # def load_waypoints(self, waypoints=None, suppress_output=False):
    #     # A waypoint consists of a field intensity (mT) and a field angle relative to vertical (deg)
    #     # Whitespace is the delimiter
    #     # Each row represents a new waypoint

    #     self.pause_flag = True
    #     time.sleep(0.05)
    #     self.flush_input()
    #     os.system('clear')
    #     if waypoints: # waypoint file name provided as argument
    #         self.waypoints = np.loadtxt(waypoints)
    #     else: # waypoint file name not provided
    #         user_input = input('Enter waypoints file name, with extension:\n')
    #         try:
    #             self.waypoints = np.loadtxt(user_input)
    #         except (IOError, OSError):
    #             print('file not found.')

    #     self.waypoint_counter = 0
    #     if not suppress_output:
    #         print(self.waypoints)
    #         print('loaded waypoints. Returning to control in 2 seconds...')
    #         time.sleep(2)
    #         self.draw_interface()

    #     self.pause_flag = False

    # def set_waypoint(self, wp, relative=False, ):
    #     '''
    #     Set the requested waypoint as the set point.
    #         Parameters:
    #             wp (int): the waypoint number to navigate to
    #             relative (bool): whether wp should be interpreted as a relative change from the current waypoint (default False)
    #         Returns:
    #             None
    #     '''
    #     raise NotImplementedError

    # def next_waypoint(self):
    #     # Do nothing if there's no matfile or the system isn't zeroed
    #     if not self.is_matfile or not self.is_zeroed:
    #         return

    #     num_waypoints = self.waypoints.shape[0]

    #     # Increment waypoint counter
    #     self.waypoint_counter += 1

    #     if self.waypoint_counter <= num_waypoints: # not past the last waypoint
    #         # Use the mat file variables to determine the magnet angle and separation
    #         desired_field = self.waypoints[self.waypoint_counter - 1, 0]
    #         desired_angle = self.waypoints[self.waypoint_counter - 1, 1]

    #         desired_r = np.interp(desired_field, self.mat_B_rev, self.mat_r_rev) # use reversed because xp must be increasing as per np.interp documetation

    #         # Absolute step target - linear axis
    #         step_a = self.motor_step_0[0] + self.distance_to_steps(desired_r - self.motor_pos_0[0])
    #         step_d = self.motor_step_0[3] + self.distance_to_steps(desired_r - self.motor_pos_0[3])

    #         # Absolute step target - rotational axis
    #         step_b = self.motor_step_0[1] + self.angle_to_steps(desired_angle - self.motor_pos_0[1])
    #         step_c = self.motor_step_0[2] - self.angle_to_steps(desired_angle - self.motor_pos_0[2]) # Subtract because motor c local axis is opposite of global axis

    #         # Send actuation messages
    #         self.send_msg(mode='W', motor='a', steps=step_a)
    #         self.send_msg(mode='W', motor='b', steps=step_b)
    #         self.send_msg(mode='W', motor='c', steps=step_c)
    #         self.send_msg(mode='W', motor='d', steps=step_d)
    #     else: # past the last waypoint
    #         self.waypoint_counter -= 1

    # def prev_waypoint(self):
    #     # Do nothing if there's no matfile or the system isn't zeroed
    #     if not self.is_matfile or not self.is_zeroed:
    #         return

    #     # Decrement waypoint counter
    #     self.waypoint_counter -= 1

    #     if self.waypoint_counter > 0:
    #         # Use the mat file variables to determine the magnet angle and separation
    #         desired_field = self.waypoints[self.waypoint_counter - 1, 0]
    #         desired_angle = self.waypoints[self.waypoint_counter - 1, 1]

    #         desired_r = np.interp(desired_field, self.mat_B_rev, self.mat_r_rev)  # use reversed because xp must be increasing as per np.interp documetation

    #         # Absolute step target - linear axis
    #         step_a = self.motor_step_0[0] + self.distance_to_steps(desired_r - self.motor_pos_0[0])
    #         step_d = self.motor_step_0[3] + self.distance_to_steps(desired_r - self.motor_pos_0[3])

    #         # Absolute step target - rotational axis
    #         step_b = self.motor_step_0[1] + self.angle_to_steps(desired_angle - self.motor_pos_0[1])
    #         step_c = self.motor_step_0[2] - self.angle_to_steps(desired_angle - self.motor_pos_0[2]) # Subtract because motor c local axis is opposite of global axis

    #         # Send actuation messages
    #         self.send_msg(mode='W', motor='a', steps=step_a)
    #         self.send_msg(mode='W', motor='b', steps=step_b)
    #         self.send_msg(mode='W', motor='c', steps=step_c)
    #         self.send_msg(mode='W', motor='d', steps=step_d)
    #     else: # before the first waypoint
    #         self.waypoint_counter += 1

    # def zero(self):
    #     # TODO: find proper zeroing_separation
    #     zeroing_separation = 45.75 # mm, the distance from the motor mounting face to the magnet centre. This way we can zero off the back edge of the motor mounting plate which is right next to the ruler

    #     self.pause_flag = True
    #     self.flush_input()
    #     time.sleep(0.05)
    #     os.system('clear')
    #     print('Zeroing motors. Note: motors b and c can only be zeroed at vertical position')
    #     user_input = input('What motors should be zeroed? e.g. "a", "ab", "acd", etc.\n')
    #     for i in user_input:
    #         if i == 'a':
    #             position = input('Enter motor a position in mm:\n')
    #             try:
    #                 self.motor_pos_0[0] = float(position) - zeroing_separation
    #                 self.motor_step_0[0] = self.latest_motor_steps[0]
    #             except ValueError:
    #                 print('Invalid value {}, skipping...'.format(position))
    #         elif i == 'b':
    #             self.motor_pos_0[1] = 0
    #             self.motor_step_0[1] = self.latest_motor_steps[1]
    #             print('motor b zeroed')
    #         elif i == 'c':
    #             self.motor_pos_0[2] = 0
    #             self.motor_step_0[2] = self.latest_motor_steps[2]
    #             print('motor c zeroed')
    #         elif i == 'd':
    #             position = input('Enter motor d position in mm:\n')
    #             try:
    #                 self.motor_pos_0[3] = float(position) - zeroing_separation
    #                 self.motor_step_0[3] = self.latest_motor_steps[3]
    #             except ValueError:
    #                 print('Invalid value {}, skipping...'.format(position))
    #         else:
    #             print('Invalid value {}, skipping...'.format(i))

    #     self.is_zeroed = True
    #     print('System zeroed. Check reported position vs actual to confirm.')
    #     print('Returning to control...')
    #     time.sleep(1)
    #     self.draw_interface()
    #     self.pause_flag = False

