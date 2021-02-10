import serial
import numpy as np
import os
import time

from PyQt5 import uic
from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import QMainWindow

from subthreads import ArduinoManager

#=========================================================
# UI Setup
#=========================================================
qtCreatorFile = 'doublemag.ui'
Ui_MainWindow, QtBaseClass = uic.loadUiType(qtCreatorFile)

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


class DoubleMagnetGUI(QtGui.QMainWindow, Ui_MainWindow):
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

        # Create serial connection
        self.ser = serial.Serial(ARDUINO_PORT, ARDUINO_BAUD, timeout=1)

        # Variables to track motor position
        self.motor_step_0 = np.array([0., 0., 0., 0.])
        self.motor_step = np.array([0., 0., 0., 0.])
        self.motor_pos_0 = np.array([0., 0., 0., 0.])
        self.motor_pos = np.array([0., 0., 0., 0.])

        # Variables to track magnetic reading
        self.mag = np.array([0., 0., 0.]) # x, y, z (mT)

    def setupTimer(self):
        self.timer = QTimer()
        self.timer.timeout.connect(self.update)
        self.timer.start(1000.0/30.0) # milliseconds between updates

    def update(self):
        # Get latest readings
        self.updateReadings()

        # Send motor commands
        # TODO:

        # Update UI
        # TODO:
        raise NotImplementedError

    def updateReadings(self):
        while self.ser.in_waiting > 0: # Work through buffer until most recent message
            incoming_serial_words = self.ser.readline().decode().rstrip().split(',')

            # Check the first entry in values to see what kind of data it is
            try:
                if incoming_serial_words[0] == 'steps':
                    # Assumed message contents:
                    # 0: 'steps'
                    # 1-4: step value of motor A, B, C, D
                    # 5-7: Magnetic field in x, y, z
                    try:
                        # Update position
                        self.motor_step[:] = np.array([float(x) for x in incoming_serial_words[1:5]])
                        stepdiff = self.motor_step - self.motor_step_0
                        self.motor_pos[0] = self.motor_pos_0[0] + self.steps_to_distance(stepdiff[0])
                        self.motor_pos[1] = self.motor_pos_0[1] + self.steps_to_angle(stepdiff[1])
                        self.motor_pos[2] = -(self.motor_pos_0[2] + self.steps_to_angle(stepdiff[2]))
                        self.motor_pos[3] = self.motor_pos_0[3] + self.steps_to_distance(stepdiff[3])

                        # Update magnetic
                        for i in [5, 6, 7]:
                            self.mag[i] = incoming_serial_words[i]
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
            except NameError:
                # No messages?
                pass


    def load_waypoints(self, waypoints=None, suppress_output=False):
        # A waypoint consists of a field intensity (mT) and a field angle relative to vertical (deg)
        # Whitespace is the delimiter
        # Each row represents a new waypoint

        self.pause_flag = True
        time.sleep(0.05)
        self.flush_input()
        os.system('clear')
        if waypoints: # waypoint file name provided as argument
            self.waypoints = np.loadtxt(waypoints)
        else: # waypoint file name not provided
            user_input = input('Enter waypoints file name, with extension:\n')
            try:
                self.waypoints = np.loadtxt(user_input)
            except (IOError, OSError):
                print('file not found.')

        self.waypoint_counter = 0
        if not suppress_output:
            print(self.waypoints)
            print('loaded waypoints. Returning to control in 2 seconds...')
            time.sleep(2)
            self.draw_interface()

        self.pause_flag = False

    def set_waypoint(self, wp, relative=False, ):
        '''
        Set the requested waypoint as the set point.
            Parameters:
                wp (int): the waypoint number to navigate to
                relative (bool): whether wp should be interpreted as a relative change from the current waypoint (default False)
            Returns:
                None
        '''
        raise NotImplementedError

    def next_waypoint(self):
        # Do nothing if there's no matfile or the system isn't zeroed
        if not self.is_matfile or not self.is_zeroed:
            return

        num_waypoints = self.waypoints.shape[0]

        # Increment waypoint counter
        self.waypoint_counter += 1

        if self.waypoint_counter <= num_waypoints: # not past the last waypoint
            # Use the mat file variables to determine the magnet angle and separation
            desired_field = self.waypoints[self.waypoint_counter - 1, 0]
            desired_angle = self.waypoints[self.waypoint_counter - 1, 1]

            desired_r = np.interp(desired_field, self.mat_B_rev, self.mat_r_rev) # use reversed because xp must be increasing as per np.interp documetation

            # Absolute step target - linear axis
            step_a = self.motor_step_0[0] + self.distance_to_steps(desired_r - self.motor_pos_0[0])
            step_d = self.motor_step_0[3] + self.distance_to_steps(desired_r - self.motor_pos_0[3])

            # Absolute step target - rotational axis
            step_b = self.motor_step_0[1] + self.angle_to_steps(desired_angle - self.motor_pos_0[1])
            step_c = self.motor_step_0[2] - self.angle_to_steps(desired_angle - self.motor_pos_0[2]) # Subtract because motor c local axis is opposite of global axis

            # Send actuation messages
            self.send_msg(mode='W', motor='a', steps=step_a)
            self.send_msg(mode='W', motor='b', steps=step_b)
            self.send_msg(mode='W', motor='c', steps=step_c)
            self.send_msg(mode='W', motor='d', steps=step_d)
        else: # past the last waypoint
            self.waypoint_counter -= 1

    def prev_waypoint(self):
        # Do nothing if there's no matfile or the system isn't zeroed
        if not self.is_matfile or not self.is_zeroed:
            return

        # Decrement waypoint counter
        self.waypoint_counter -= 1

        if self.waypoint_counter > 0:
            # Use the mat file variables to determine the magnet angle and separation
            desired_field = self.waypoints[self.waypoint_counter - 1, 0]
            desired_angle = self.waypoints[self.waypoint_counter - 1, 1]

            desired_r = np.interp(desired_field, self.mat_B_rev, self.mat_r_rev)  # use reversed because xp must be increasing as per np.interp documetation

            # Absolute step target - linear axis
            step_a = self.motor_step_0[0] + self.distance_to_steps(desired_r - self.motor_pos_0[0])
            step_d = self.motor_step_0[3] + self.distance_to_steps(desired_r - self.motor_pos_0[3])

            # Absolute step target - rotational axis
            step_b = self.motor_step_0[1] + self.angle_to_steps(desired_angle - self.motor_pos_0[1])
            step_c = self.motor_step_0[2] - self.angle_to_steps(desired_angle - self.motor_pos_0[2]) # Subtract because motor c local axis is opposite of global axis

            # Send actuation messages
            self.send_msg(mode='W', motor='a', steps=step_a)
            self.send_msg(mode='W', motor='b', steps=step_b)
            self.send_msg(mode='W', motor='c', steps=step_c)
            self.send_msg(mode='W', motor='d', steps=step_d)
        else: # before the first waypoint
            self.waypoint_counter += 1

    def zero(self):
        # TODO: find proper zeroing_separation
        zeroing_separation = 45.75 # mm, the distance from the motor mounting face to the magnet centre. This way we can zero off the back edge of the motor mounting plate which is right next to the ruler

        self.pause_flag = True
        self.flush_input()
        time.sleep(0.05)
        os.system('clear')
        print('Zeroing motors. Note: motors b and c can only be zeroed at vertical position')
        user_input = input('What motors should be zeroed? e.g. "a", "ab", "acd", etc.\n')
        for i in user_input:
            if i == 'a':
                position = input('Enter motor a position in mm:\n')
                try:
                    self.motor_pos_0[0] = float(position) - zeroing_separation
                    self.motor_step_0[0] = self.latest_motor_steps[0]
                except ValueError:
                    print('Invalid value {}, skipping...'.format(position))
            elif i == 'b':
                self.motor_pos_0[1] = 0
                self.motor_step_0[1] = self.latest_motor_steps[1]
                print('motor b zeroed')
            elif i == 'c':
                self.motor_pos_0[2] = 0
                self.motor_step_0[2] = self.latest_motor_steps[2]
                print('motor c zeroed')
            elif i == 'd':
                position = input('Enter motor d position in mm:\n')
                try:
                    self.motor_pos_0[3] = float(position) - zeroing_separation
                    self.motor_step_0[3] = self.latest_motor_steps[3]
                except ValueError:
                    print('Invalid value {}, skipping...'.format(position))
            else:
                print('Invalid value {}, skipping...'.format(i))

        self.is_zeroed = True
        print('System zeroed. Check reported position vs actual to confirm.')
        print('Returning to control...')
        time.sleep(1)
        self.draw_interface()
        self.pause_flag = False

    def steps_to_distance(self, steps):
        return (steps / STEPS_PER_REV) * MM_PER_REV

    def distance_to_steps(self, position):
        return (position /MM_PER_REV) * STEPS_PER_REV

    def steps_to_angle(self, steps):
        return (steps / STEPS_PER_REV) * 360.0

    def angle_to_steps(self, angle):
        return (angle / 360.0) * STEPS_PER_REV

    def draw_position(self):
        # Draw the indications of each magnet position
        # motor_pos is a list containing the real unit positions of the motors.
        # m1 and m4 are linear position (mm)
        # m2 and m3 are angular position (deg)
        # This should be called every time an update from the arduino is available. It's supposed to just update the most recent line with the new position for a live view.

        # if the motors are symmetric enough, calculate and display the field
        tol = 0.5 # Angular and mm tolerance to define symmetry
        if abs(self.latest_motor_pos[0] - self.latest_motor_pos[3]) < tol and abs(self.latest_motor_pos[1] - self.latest_motor_pos[2]) < tol and self.is_matfile and self.is_zeroed:
            r = np.mean([self.latest_motor_pos[0], self.latest_motor_pos[3]]) # No unit conversion because matfile is converted to mm/mT at load
            B = np.interp(r, self.mat_r, self.mat_B, left=np.nan, right=np.nan) # No unit conversion because matfile is converted to mm/mT at load
            if not np.isnan(B):
                print('A: {0:5.1f} mm, B: {1:4.1f} deg, C: {2:4.1f} deg, D: {3:5.1f} mm, wp: {4:d}, B: {5:4.1f} mT ###\r'.format(self.latest_motor_pos[0], self.latest_motor_pos[1], self.latest_motor_pos[2], self.latest_motor_pos[3], self.waypoint_counter, B), end="")
            else: # r is outside of the range of the matfile
                print('A: {0:5.1f} mm, B: {1:4.1f} deg, C: {2:4.1f} deg, D: {3:5.1f} mm, wp: {4:d}, B out of range ###\r'.format(self.latest_motor_pos[0], self.latest_motor_pos[1], self.latest_motor_pos[2], self.latest_motor_pos[3], self.waypoint_counter), end="")
        else:
            print('A: {0:5.1f} mm, B: {1:4.1f} deg, C: {2:4.1f} deg, D: {3:5.1f} mm, wp: {4:d}, Fixture nonsymmetric or not zeroed ###\r'.format(self.latest_motor_pos[0], self.latest_motor_pos[1], self.latest_motor_pos[2], self.latest_motor_pos[3], self.waypoint_counter), end="")

    def loop(self):
        while not self.exit_flag:
            if not self.pause_flag:
                time.sleep(0.03)
                self.parse_position()
                self.draw_position()

        # Exited the loop: stop all motors
        self.send_msg(mode='M', motor='a', steps=0)
        self.send_msg(mode='M', motor='b', steps=0)
        self.send_msg(mode='M', motor='c', steps=0)
        self.send_msg(mode='M', motor='d', steps=0)

        self.flush_input()
        time.sleep(0.05)

        # Close serial port
        self.ser.close()


