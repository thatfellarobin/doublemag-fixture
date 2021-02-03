import serial
from pynput import keyboard
import numpy as np
from scipy.io import loadmat
import os
import time

# Notes on direction
# - Positive # of steps rotates the motor shaft clockwise as viewed from the motor body.
# - For the linear axis, clockwise rotation moves the magnet away from the workspace

# Notes on units
# - fixture records position in mm

# TODO and FIXME:
# - Optionally revert to safe state upon exit if zeroed
# - Something broken when the following process was followed:
#   - Manual control > zeroing > waypoint control > manual control > waypoint control


class doubleMagController:
    def __init__(self, arduino_port='/dev/tty.usbserial-AC00921Z', baud=9600, magnet_mat=None, waypoints=None):
        # magnet_mat: MATLAB .mat filename with info on magnet position versus field
        # waypoints: whitespace-delimited file with field magnitude and orientation

        self.ser = serial.Serial(arduino_port, baud, timeout=1) # timeout is necessary so it can wait a reasonable amount of time for a new line to arrive from the arduino. if timeout=0 then it reads partial lines because it's too fast
        self.exit_flag = False
        self.latest_motor_steps = np.array([0., 0., 0., 0.])
        self.motor_step_0 = np.array([0., 0., 0., 0.])
        self.latest_motor_pos = np.array([0., 0., 0., 0.])
        self.motor_pos_0 = np.array([0., 0., 0., 0.])
        self.pause_flag = False # flag to indicate whether zeroing/something else is in progress, and if so, to pause the main thread

        self.is_matfile = False
        self.is_zeroed = False

        if magnet_mat:
            # Matfile must have the following corresponding vectors:
            # - vector 'r' which is centre-to-centre distance between magnet and workspace, in m
            # - vector 'B_map_norm' which is field intensity, in T
            self.matfile = loadmat(magnet_mat)
            self.mat_r = np.squeeze(self.matfile['r']) * 1000 # Convert T to mT
            self.mat_B = np.squeeze(self.matfile['B_map_norm']) * 1000 # Convert m to mm
            self.mat_r_rev = np.flip(self.mat_r) # np.interp requires increasing xp array as per documentation, this is for that case
            self.mat_B_rev = np.flip(self.mat_B) # np.interp requires increasing xp array as per documentation, this is for that case
            self.is_matfile = True
        if waypoints:
            self.load_waypoints(waypoints=waypoints, suppress_output=True)

        self.listener = keyboard.Listener(
            on_press=self.key_press,
            on_release=self.key_release)
        self.listener.start()

        # Start controller
        self.draw_interface()
        self.loop()

    def send_msg(self, mode, motor, steps):
        # mode: 'M' or 'W' indicating manual or waypoint control, respectively
        # motor: lowercase letter representing the motor to be moved
        # steps: number of steps to move. positive or negative. Doesn't have to be int.
        #   - Note that for manual control, the magnitude doesn't matter, only the sign.
        if steps > 999999 or steps < -999999:
            raise ValueError("# steps must be between -999999 and 999999")

        if steps > 0:
            direction = '+'
        elif steps < 0:
            direction = '-'
        else:
            direction = '0'

        if mode == 'M':
            msg = mode + motor + direction + '\n'
        elif mode == 'W':
            msg = mode + motor + str(abs(int(steps))).zfill(6) + direction + '\n'
        else:
            raise ValueError("mode must be 'M' or 'W'")

        msg_encode = msg.encode(encoding='ascii')
        self.ser.write(msg_encode)

    def parse_position(self):
        if self.ser.in_waiting > 0:
            incoming_serial_words = self.ser.readline().decode().rstrip().split(',')

            # Check the first entry in values to see what kind of data it is
            if incoming_serial_words[0] == 'steps' and len(incoming_serial_words) == 5:
                try:
                    self.latest_motor_steps[:] = np.array([float(x) for x in incoming_serial_words[1:]])
                    stepdiff = self.latest_motor_steps - self.motor_step_0
                    self.latest_motor_pos[0] = self.motor_pos_0[0] + self.steps_to_distance(stepdiff[0])
                    self.latest_motor_pos[1] = self.motor_pos_0[1] + self.steps_to_angle(stepdiff[1])
                    self.latest_motor_pos[2] = -(self.motor_pos_0[2] + self.steps_to_angle(stepdiff[2]))
                    self.latest_motor_pos[3] = self.motor_pos_0[3] + self.steps_to_distance(stepdiff[3])
                except ValueError:
                    pass
        else:
            pass

    def key_press(self, key):
        if not self.pause_flag:
            try:
                char = key.char
            except AttributeError:
                # not alphanumeric
                if key == keyboard.Key.left:
                    char = 'left'
                elif key == keyboard.Key.right:
                    char = 'right'
                elif key == keyboard.Key.esc:
                    char = 'esc'
                else:
                    char = 'invalid'

            if char == 'w': # Magnet one rot away
                self.send_msg(mode='M', motor='b', steps=-1)
            elif char == 'a': # Magnet one move left
                self.send_msg(mode='M', motor='a', steps=1)
            elif char == 's': # Magnet one rot toward
                self.send_msg(mode='M', motor='b', steps=1)
            elif char == 'd': # Magnet one move right
                self.send_msg(mode='M', motor='a', steps=-1)
            elif char == 'i': # Magnet two rot away
                self.send_msg(mode='M', motor='c', steps=1)
            elif char == 'j': # Magnet two move left
                self.send_msg(mode='M', motor='d', steps=-1)
            elif char == 'k': # Magnet two rot toward
                self.send_msg(mode='M', motor='c', steps=-1)
            elif char == 'l': # Magnet two move right
                self.send_msg(mode='M', motor='d', steps=1)
            elif char == 'left': # Next waypoint
                self.prev_waypoint()
            elif char == 'right': # Previous waypoint
                self.next_waypoint()
            elif char == 'z':
                self.zero()
            elif char == 'x':
                self.load_waypoints()
            elif char == 'esc':
                self.exit_flag = True

    def key_release(self, key):
        try:
            char = key.char
        except AttributeError:
            char = 'invalid'

        if char == 'w' or char == 's':
            self.send_msg(mode='M', motor='b', steps=0)
        elif char == 'a' or char == 'd':
            self.send_msg(mode='M', motor='a', steps=0)
        elif char == 'i' or char == 'k':
            self.send_msg(mode='M', motor='c', steps=0)
        elif char == 'j' or char == 'l':
            self.send_msg(mode='M', motor='d', steps=0)

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
        microstep_factor = 8.0
        motor_steps_per_rev = 200.0
        steps_per_rev = microstep_factor * motor_steps_per_rev
        mm_per_rev = 8.0
        return (steps / steps_per_rev) * mm_per_rev

    def distance_to_steps(self, position):
        microstep_factor = 8.0
        motor_steps_per_rev = 200.0
        steps_per_rev = microstep_factor * motor_steps_per_rev
        mm_per_rev = 8.0
        return (position / mm_per_rev) * steps_per_rev

    def steps_to_angle(self, steps):
        microstep_factor = 8.0
        motor_steps_per_rev = 200.0
        steps_per_rev = microstep_factor * motor_steps_per_rev
        return (steps / steps_per_rev) * 360.0

    def angle_to_steps(self, angle):
        microstep_factor = 8.0
        motor_steps_per_rev = 200.0
        steps_per_rev = microstep_factor * motor_steps_per_rev
        return (angle / 360.0) * steps_per_rev

    def flush_input(self):
        # From https://stackoverflow.com/questions/24582233/python-flush-input-before-raw-input/24618062
        try:
            import msvcrt
            while msvcrt.kbhit():
                msvcrt.getch()
        except ImportError:
            import sys
            import termios
            termios.tcflush(sys.stdin, termios.TCIOFLUSH)

    def draw_interface(self):
        # Draw the text interface
        # Draw the controls to rotate and move each magnet
        # Indicate what buttons to press for other functions: zeroing, next waypoint, previous waypoint
        # This should get called only once when the interface needs to be brought back up. Otherwise it should not be called repetitively.

        print('    W      <-rot up->      I    ')
        print('    ^                      ^    ')
        print('A <---> D              J <---> L')
        print('    v                      v    ')
        print('    S      <-rot dn->      K    ')
        print()
        print('Load Waypoint: < X >            ')
        print('Zero position: < Z >            ')
        print('Next Waypoint: < Right >        ')
        print('Prev Waypoint: < Left >         ')
        print('Exit:          < Esc >          ')
        print()

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

        # Stop all motors
        self.send_msg(mode='M', motor='a', steps=0)
        self.send_msg(mode='M', motor='b', steps=0)
        self.send_msg(mode='M', motor='c', steps=0)
        self.send_msg(mode='M', motor='d', steps=0)

        self.flush_input()
        time.sleep(0.05)

        # Close serial port
        self.ser.close()


if __name__ == '__main__':
    controller = doubleMagController(magnet_mat='two_mag_constangle_field_17.27Am2.mat', waypoints='waypoints_angle_30.txt')
