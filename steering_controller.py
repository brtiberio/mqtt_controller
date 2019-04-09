#!/usr/bin/python
# -*- coding: utf-8 -*-
# The MIT License (MIT)
# Copyright (c) 2018 Bruno Tibério
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import logging
import sys
import threading
import time
import gc

import numpy as np
import canopen
import pathlib
import csv
from EPOS_Canopen.epos import Epos
# ------------------------------------------------------------------------------
# Pycharm remote debug settings
# ------------------------------------------------------------------------------
# import pydevd

# pydevd.settrace("10.42.0.1", port=8000, stdoutToServer=True, stderrToServer=True)


# shortcut for clear console


# def clear():
#     os.system('cls' if os.name == 'nt' else 'clear')

# This code section is based on
# https://codereview.stackexchange.com/questions/32207/console-user-main-menu
class Menu(object):
    """Base class for the menu"""

    def __init__(self, name, buttons):
        # Initialize values
        self.name = name
        self.buttons = buttons
        self.exitFlag = False

    def display(self):
        """Displaying the menu alongside the navigation elements"""

        # clear()
        gc.collect()  # garbage collect
        # Display menu name
        print(self.name)

        # Display menu buttons
        for button in self.buttons:
            print("   ", button.nav, button.name)

        # Wait for user input
        return self.userInput()

    def userInput(self):
        """Method to check and act upon user's input"""

        # This holds the amount of errors for the
        # navigation element to input comparison.
        errSel = 0
        inputSel = input("Enter selection> ")

        for button in self.buttons:
            # If input equals to button's navigation element
            if inputSel == str(button.nav):
                # Do the button's function
                return button.nav
            # If input != navigation element
            else:
                # Increase "error on selection" by one, for
                # counting the errors and checking their
                # amount against the total number of
                # buttons. If greater to or equal that means
                # no elements were selected.
                # In that case show error and try again
                errSel += 1

        # No usable input, try again
        if errSel >= len(self.buttons):
            print("Error on selection; try again.")
            return None


class Button(object):
    """Base class for menu buttons"""

    def __init__(self, name, nav):
        # Initialize values
        self.name = name
        # Navigation element; number which user has to enter to do button action
        self.nav = nav


class Epos_controller(Epos):
    maxFollowingError = 7500
    minValue = 0  # type: int
    maxValue = 0  # type: int
    zeroRef = 0  # type: int
    calibrated = 0  # type: bool
    QC_TO_DELTA = -7.501E-4  # type: float
    DELTA_TO_QC = 1.0 / QC_TO_DELTA  # type: float
    maxAngle = 29  # type: int
    minAngle = -maxAngle
    dataDir = "./data/"  # type: str
    errorDetected = False  # type: bool
    emcy_descriptions = [
        # Code   Description
        (0x0000, "Error Reset / No Error"),
        (0x1000, "Generic Error"),
        (0x2310, "Over Current Error"),
        (0x3210, "Over Voltage Error"),
        (0x3220, "Under Voltage"),
        (0x4210, "Over Temperature"),
        (0x5113, "Supply Voltage (+5V) too low"),
        (0x6100, "Internal Software Error"),
        (0x6320, "Software Parameter Error"),
        (0x7320, "Sensor Position Error"),
        (0x8110, "CAN Overrun Error (Objects lost)"),
        (0x8111, "CAN Overrun Error"),
        (0x8120, "CAN Passive Mode Error"),
        (0x8130, "CAN Life Guard Error"),
        (0x8150, "CAN Transmit COB-ID collision"),
        (0x81FD, "CAN Bus Off"),
        (0x81FE, "CAN Rx Queue Overrun"),
        (0x81FF, "CAN Tx Queue Overrun"),
        (0x8210, "CAN PDO length Error"),
        (0x8611, "Following Error"),
        (0x9000, "External Error"),
        (0xF001, "Hall Sensor Error"),
        (0xFF02, "Index Processing Error"),
        (0xFF03, "Encoder Resolution Error"),
        (0xFF04, "Hallsensor not found Error"),
        (0xFF06, "Negative Limit Error"),
        (0xFF07, "Positive Limit Error"),
        (0xFF08, "Hall Angle detection Error"),
        (0xFF09, "Software Position Limit Error"),
        (0xFF0A, "Position Sensor Breach"),
        (0xFF0B, "System Overloaded")
    ]

    def emcy_error_print(self, emcy_error):
        """Print any EMCY Error Received on CAN BUS
        """
        if emcy_error.code is 0:
            self.errorDetected = False
        else:
            for code, description in self.emcy_descriptions:
                if emcy_error.code == code:
                    self.errorDetected = True
                    self.log_info("Got an EMCY message: Code: 0x{0:04X} {1}".format(code, description))
                    return
            # if no description was found, print generic info
            self.errorDetected = True
            self.log_info('Got an EMCY message: {0}'.format(emcy_error))
        return

    def get_qc_position(self, delta):
        """ Converts angle of wheels to qc

        Given the desired angle of wheels, in degrees of the bicycle model of car,
        convert the requested value to qc position of steering wheel using the
        calibration performed at beginning.

        Args:
            delta: desired angle of wheels in degrees.
        Returns:
            int: a rounded integer with qc position estimated or None if not possible
        """
        if not self.calibrated:
            self.log_info('Device is not yet calibrated')
            return None
        if delta > self.maxAngle:
            self.log_info('Angle exceeds limits: maxAngle: {0}\t requested: {1}'.format(
                self.maxAngle,
                delta))
            return None
        if delta < self.minAngle:
            self.log_info('Angle exceeds limits: minAngle: {0}\t requested: {1}'.format(
                self.minAngle,
                delta))
            return None
        # perform calculations y = mx + b
        val = delta * self.DELTA_TO_QC + self.zeroRef
        val = round(val)
        return int(val)

    def get_delta_angle(self, qc):
        """ Converts qc of steering wheel to angle of wheel

        Given the desired qc steering position, convert the requested value to angle of bicycle model in degrees.

        Args:
            qc: an int with desired qc position of steering wheel.
        Returns:
            double: estimated angle of wheels in degrees or None if not possible
        """
        if not self.calibrated:
            self.log_info('Device is not yet calibrated')
            return None

        # perform calculations y = mx + b and solve to x
        delta = (qc - self.zeroRef) * self.QC_TO_DELTA
        return float(delta)

    def save_to_file(self, filename=None, exitFlag=None):
        """Record qc positions into a csv file

        The following fields will be recorded

        +-------+----------+-------+
        | time  | position | angle |
        +-------+----------+-------+
        | t1    | p1       | a1    |
        +-------+----------+-------+
        | ...   | ...      | ...   |
        +-------+----------+-------+
        | tN    | pN       | aN    |
        +-------+----------+-------+

        An additional file with same name but with ext TXT will have the current
        calibration parameters

         * minValue
         * maxValue
         * zeroRef

        If filename is not supplied or already used, the current asctime() will
        be used as filename.

        Args:
            filename: name of the file to save the data
            exitFlag: threading event flag to signal exit.
        """
        # check if inputs were supplied
        if not exitFlag:
            self.log_info('Error: exitFlag must be supplied')
            return
        # make sure is clear.
        if exitFlag.isSet():
            exitFlag.clear()
        # ----------------------------------------------------------------------
        # Confirm epos is in a suitable state for free movement
        # ----------------------------------------------------------------------
        stateID = self.check_state()
        # failed to get state?
        if stateID is -1:
            self.log_info('Error: Unknown state')
            return
        # If epos is not in disable operation at least,
        # motor is expected to be blocked
        if stateID > 4:
            self.log_info('Not a proper operation mode: {0}'.format(
                self.state[stateID]))
            if not self.change_state('shutdown'):
                self.log_info('Failed to change Epos state to shutdown')
                return
            self.log_info('Successfully changed Epos state to shutdown')
        # all ok, proceed
        if not filename:
            filename = time.asctime()
            # windows has a problem with ':' in the name of files
            # replace it by underscores
            filename = filename.replace(':', '_')
            filename = filename.replace(' ', '_')

        # make dir if not already made
        pathlib.Path(self.dataDir).mkdir(parents=True, exist_ok=True)
        my_file = pathlib.Path(self.dataDir + filename + '.csv')

        # open the parameters file first
        my_file = open(self.dataDir + filename + '.txt', 'w')
        print("minValue = {0}\nmaxValue = {1}\nzeroRef = {2}".format(self.minValue,
                                                                     self.maxValue, self.zeroRef), file=my_file,
              flush=True)
        my_file.close()

        # open the csv file
        my_file = open(self.dataDir + filename + '.csv', 'w')
        writer = csv.DictWriter(my_file, fieldnames=[
            'time', 'position', 'angle'])
        writer.writeheader()
        # -----------------------------------------------------------------------
        # start requesting for positions of sensor
        # -----------------------------------------------------------------------
        # var to store number of fails
        num_fails = 0
        # get current time
        t0 = time.monotonic()
        while not exitFlag.isSet():
            current_value, ok = self.read_position_value()
            tOut = time.monotonic() - t0
            if not ok:
                self.log_info('Failed to request current position')
                num_fails = num_fails + 1
            else:
                writer.writerow({'time': tOut, 'position': current_value,
                                 'angle': self.get_delta_angle(current_value)})
            # sleep?
            time.sleep(0.01)

        self.log_info('Finishing collecting data with {0} fail readings'.format(
            num_fails))
        my_file.close()

    def read_from_file(self, filename=None, use_angle=False):
        """Read qc positions from file and follow them

        The file must contain time and position in quadrature positions of steering
        wheel and angle (degrees) of "center" wheel of bicycle model in a csv style

        +-------+----------+-------+
        | time  | position | angle |
        +-------+----------+-------+
        | t1    | p1       | a1    |
        +-------+----------+-------+
        | ...   | ...      | ...   |
        +-------+----------+-------+
        | tN    | pN       | aN    |
        +-------+----------+-------+

        If the calibration value is not the same, user should set useAngle flag.
        Because the calibration when the file was created can differ from current
        calibration, user should set the useAngle flag to calculate the position
        reference to send for device via the given angle.

        Args:
            filename: csv file to be read.
            use_angle: use the angle value instead of position.
        """
        self.log_info('Filename is {0}'.format(filename))
        # get current state of epos
        state = self.check_state()
        if state is -1:
            self.log_info('Error: Unknown state')
            return

        if state is 11:
            # perform fault reset
            if not self.change_state('fault reset'):
                self.log_info('Error: Failed to change state to fault reset')
                return
        # get current op mode
        op_mode, ok = self.read_op_mode()
        if not ok:
            logging.info('Failed to request current OP Mode')
            return
        # show current op Mode
        self.log_info('Current OP Mode is {0}'.format(
            self.opModes[op_mode]
        ))
        # check if mode is position
        if op_mode is not -1:
            if not self.set_op_mode(-1):
                self.log_info('Failed to change op_mode to {0}'.format(
                    self.opModes[-1]
                ))
                return
            else:
                self.log_info('OP Mode is now {0}'.format(
                    self.opModes[-1]
                ))
        # shutdown
        if not self.change_state('shutdown'):
            self.log_info('Failed to change Epos state to shutdown')
            return
        # switch on
        if not self.change_state('switch on'):
            self.log_info('Failed to change Epos state to switch on')
            return
        if not self.change_state('enable operation'):
            self.log_info('Failed to change Epos state to enable operation')
            return

        # check if file exist
        my_file = pathlib.Path(filename)
        if not my_file.exists():
            self.log_info('File does not exist: {0}'.format(
                my_file))
            return

        # open csv file and read all values.
        with open(filename) as csvfile:
            reader = csv.DictReader(csvfile, delimiter=',')
            I = 0  # line number
            for row in reader:
                t_target = float(row['time'])  # type: float
                if use_angle:
                    angle = float(row['angle'])
                    if angle is not None:  # if angle exceed limits, do not update
                        position = self.get_qc_position(angle)
                else:
                    position = int(row['position'])

                # align to the first position before starting
                if I is 0:
                    self.move_to_position(position)
                    try:
                        input("Press any key when ready...")
                    except KeyboardInterrupt as e:
                        self.log_info('Got exception {0}... exiting now'.format(e))
                        # shutdown
                        if not self.change_state('shutdown'):
                            self.log_info('Failed to change Epos state to shutdown')
                        return
                    num_fails = 0
                    t0 = time.monotonic()
                    last_read = 0
                else:
                    # if is not the first position but tOut is not yeat t_target
                    # sleep
                    # skip to next step?
                    while True:
                        tOut = time.monotonic() - t0
                        # is time to send new values?
                        if tOut > t_target:
                            # time to update
                            if not self.set_position_mode_setting(position):
                                num_fails = num_fails + 1
                            break
                        # if we are not sending new targets, request current value to see the error
                        if tOut - last_read > 0.051:
                            aux, OK = self.read_position_value()
                            if not OK:
                                self.log_info('Failed to request current position')
                                num_fails = num_fails + 1
                            else:
                                # does error have grown to much?
                                ref_error = position - aux
                                if abs(ref_error) > self.maxFollowingError:
                                    self.log_info(
                                        'Error is growing to much. Something seems wrong')
                                    print('time={0:+08.3f}\tIn={1:+05}\tOut={2:+05}\tError={3:+05}'.format(
                                        tOut, position, aux, ref_error))
                                    if not self.change_state('shutdown'):
                                        self.log_info(
                                            'Failed to change Epos state to shutdown')
                                    return
                                # for debug print every time on each cycle
                                print('time={0:+08.3f}\tIn={1:+05}\tOut={2:+05}\tError={3:+05}'.format(
                                    tOut, position, aux, ref_error))
                            last_read = tOut
                        time.sleep(0.001)
                I = I + 1  # increase line number
                if self.errorDetected:
                    break

            if self.errorDetected:
                self.log_info('Exited with emergency error')
            else:
                self.log_info('All done: Time to process all vars was {0} seconds with {1} fail readings'.format(
                    time.monotonic() - t0, num_fails))
                self.log_info('Expected time to process {0}'.format(t_target))
            if not self.change_state('shutdown'):
                self.log_info('Failed to change Epos state to shutdown')
            return

    def start_calibration(self, exit_flag=None):
        """Perform steering wheel calibration

        This function is expected to be run on a thread in order to find the limits
        of the steering wheel position and find the expected value of the zero angle
        of wheels.

        Args:
            exit_flag: threading.Event() to signal the finish of acquisition

        """
        # check if inputs were supplied
        if not exit_flag:
            self.log_info('Error: check arguments supplied')
            return
        state_id = self.check_state()
        # -----------------------------------------------------------------------
        # Confirm epos is in a suitable state for free movement
        # -----------------------------------------------------------------------
        # failed to get state?
        if state_id is -1:
            self.log_info('Error: Unknown state')
            return
        # If epos is not in disable operation at least, motor is expected to be blocked
        if state_id > 4:
            self.log_info('Not a proper operation mode: {0}'.format(
                self.state[state_id]))
            # shutdown
            if not self.change_state('shutdown'):
                self.log_info('Failed to change state to shutdown')
                return
            self.log_info('Successfully changed state to shutdown')

        max_value = 0
        min_value = 0
        num_fails = 0
        # -----------------------------------------------------------------------
        # start requesting for positions of sensor
        # -----------------------------------------------------------------------
        while not exit_flag.isSet():
            current_value, ok = self.read_position_value()
            if not ok:
                self.log_debug('Failed to request current position')
                num_fails = num_fails + 1
            else:
                if current_value > max_value:
                    max_value = current_value
                if current_value < min_value:
                    min_value = current_value
            # sleep?
            time.sleep(0.01)

        self.log_info(
            'Finished calibration routine with {0} fail readings'.format(num_fails))
        self.minValue = min_value
        self.maxValue = max_value
        self.zeroRef = round((max_value - min_value) / 2.0 + min_value)
        self.calibrated = 1
        self.log_info('MinValue: {0}, MaxValue: {1}, ZeroRef: {2}'.format(
            self.minValue, self.maxValue, self.zeroRef
        ))
        return

    def move_to_position(self, pos_final, is_angle=False):
        """Move to desired position.

        Plan and apply a motion profile to reduce with low jerk, max speed, max acceleration
        to avoid abrupt variations.
        The function implement the algorithm developed in [1]_

        Args:
            pos_final: desired position.
            is_angle: a boolean, true if pos_final is an angle or false if is qc value
        :return:

        .. [1] Li, Huaizhong & M Gong, Z & Lin, Wei & Lippa, T. (2007). Motion profile planning for reduced jerk and vibration residuals. 10.13140/2.1.4211.2647.
        """
        # constants
        # Tmax = 1.7 seems to be the limit before oscillations.
        Tmax = 0.2  # max period for 1 rotation;
        # 1 rev = 3600*4 [qc]
        countsPerRev = 3600 * 4
        #
        # 1Hz = 60rpm = 360degrees/s
        #
        # 360 degrees = sensor resolution * 4
        #
        # this yields: 1Hz = (sensor resolution * 4)/s
        #
        # Fmax = 1 / Tmax;
        #
        # maxSpeed = 60 rpm/Tmax [rpm]=
        #          = 360degrees/Tmax [degrees/s]=
        #          = (sensor resolution *4)/Tmax [qc/s]

        maxSpeed = countsPerRev / Tmax  # degrees per sec

        # max acceleration must be experimental obtained.
        # reduced and fixed.
        maxAcceleration = 6000.0  # [qc]/s^2

        # maximum interval for both the acceleration  and deceleration phase are:
        T1max = 2.0 * maxSpeed / maxAcceleration  # type: float

        # the max distance covered by these two phase (assuming acceleration equal
        # deceleration) is 2* 1/4 * Amax * T1max^2 = 1/2 * Amax * T1max^2 = 2Vmax^2/Amax
        maxL13 = 2.0 * maxSpeed ** 2 / maxAcceleration  # type: float

        # max error in quadrature counters
        MAXERROR = 7500
        numFails = 0
        # is device calibrated?
        if not self.calibrated:
            self.log_info('Device is not yet calibrated')
            return False
        # is position requested an angle?
        if is_angle:
            pos_final = self.get_qc_position(pos_final)
            # if position can not be calculated, alert user.
            if pos_final is None:
                self.log_info('Failed to calculate position value')
                if not self.change_state('shutdown'):
                    self.log_info('Failed to change Epos state to shutdown')
                return False

        if pos_final > self.maxValue or pos_final < self.minValue:
            self.log_info('Final position exceeds physical limits')
            return False

        pStart, OK = self.read_position_value()
        numFails = 0
        if not OK:
            self.log_info('Failed to request current position')
            while numFails < 5 and not OK:
                pStart, OK = self.read_position_value()
                if not OK:
                    numFails = numFails + 1
            if numFails == 5:
                self.log_info(
                    'Failed to request current position for 5 times... exiting')
                return False

        # -----------------------------------------------------------------------
        # get current state of epos and change it if necessary
        # -----------------------------------------------------------------------
        state = self.check_state()
        if state is -1:
            self.log_info('Error: Unknown state')
            return False

        if state is 11:
            # perform fault reset
            ok = self.change_state('fault reset')
            if not ok:
                self.log_info('Error: Failed to change state to fault reset')
                return False

        # shutdown
        if not self.change_state('shutdown'):
            self.log_info('Failed to change Epos state to shutdown')
            return False
        # switch on
        if not self.change_state('switch on'):
            self.log_info('Failed to change Epos state to switch on')
            return False
        if not self.change_state('enable operation'):
            self.log_info('Failed to change Epos state to enable operation')
            return False
        # -----------------------------------------------------------------------
        # Find remaining constants
        # -----------------------------------------------------------------------
        # absolute of displacement
        l = abs(pos_final - pStart)
        if l is 0:
            # already in final point
            return True
        # do we need  a constant velocity phase?
        if l > maxL13:
            T2 = 2.0 * (l - maxL13) / (maxAcceleration * T1max)
            T1 = T1max
            T3 = T1max
        else:
            T1 = np.sqrt(2 * l / maxAcceleration)
            T2 = 0.0
            T3 = T1

        # time constants
        t1 = T1
        t2 = T2 + t1
        t3 = T3 + t2  # final time

        # allocate vars
        inVar = np.array([], dtype='int32')
        outVar = np.array([], dtype='int32')
        tin = np.array([], dtype='int32')
        tout = np.array([], dtype='int32')
        ref_error = np.array([], dtype='int32')

        # determine the sign of movement
        moveUp_or_down = np.sign(pos_final - pStart)
        flag = True
        pi = np.pi
        cos = np.cos
        time.sleep(0.01)
        # choose monotonic for precision
        t0 = time.monotonic()
        numFails = 0
        while flag and not self.errorDetected:
            # request current time
            tin = np.append(tin, [time.monotonic() - t0])
            # time to exit?
            if tin[-1] > t3:
                flag = False
                inVar = np.append(inVar, [pos_final])
                self.set_position_mode_setting(pos_final)
                # reading a position takes time, as so, it should be enough
                # for it reaches end value since steps are expected to be
                # small
                aux, OK = self.read_position_value()
                if not OK:
                    self.log_info('Failed to request current position')
                    numFails = numFails + 1
                else:
                    outVar = np.append(outVar, [aux])
                    tout = np.append(tout, [time.monotonic() - t0])
                    ref_error = np.append(ref_error, [inVar[-1] - outVar[-1]])
            # not finished
            else:
                # get reference position for that time
                if tin[-1] <= t1:
                    aux = pStart + \
                          moveUp_or_down * maxAcceleration / 2.0 * (T1 / (2.0 * pi)) ** 2 * \
                          (1 / 2.0 * (2.0 * pi / T1 *
                                      tin[-1]) ** 2 - (1.0 - cos(2.0 / T1 * pi * tin[-1])))
                else:
                    if (T2 > 0 and tin[-1] > t1 and tin[-1] <= t2):
                        aux = pStart + \
                              moveUp_or_down * \
                              (1 / 4.0 * maxAcceleration * T1 ** 2 + 1 /
                               2.0 * maxAcceleration * T1 * (tin[-1] - t1))
                    else:
                        aux = pStart + \
                              moveUp_or_down * (1 / 4.0 * maxAcceleration * T1 ** 2
                                                + 1 / 2.0 * maxAcceleration * T1 * T2 +
                                                maxAcceleration / 2.0 *
                                                (T1 / (2.0 * pi)) ** 2
                                                * ((2.0 * pi) ** 2 * (tin[-1] - t2) / T1 - 1 / 2.0 * (2.0 * pi / T1
                                                                                                      * (tin[
                                                                                                             -1] - t2)) ** 2 + (
                                                           1.0 - cos(2.0 * pi / T1 * (tin[-1] - t2)))))
                aux = round(aux)
                # append to array and send to device
                inVar = np.append(inVar, [aux])
                OK = self.set_position_mode_setting(np.int32(inVar[-1]).item())
                if not OK:
                    self.log_info('Failed to set target position')
                    numFails = numFails + 1
                aux, OK = self.read_position_value()
                if not OK:
                    self.log_info('Failed to request current position')
                    numFails = numFails + 1
                else:
                    outVar = np.append(outVar, [aux])
                    tout = np.append(tout, [time.monotonic() - t0])
                    ref_error = np.append(ref_error, [inVar[-1] - outVar[-1]])
                    if abs(ref_error[-1]) > MAXERROR:
                        self.change_state('shutdown')
                        self.log_info(
                            'Something seems wrong, error is growing to mutch!!!')
                        return False
            # require sleep?
            time.sleep(0.005)
        self.log_info('Finished with {0} fails'.format(numFails))
        return True


def main():
    """EPOS controller tester.

    Simple program to perform a few tests with EPOS device in the car
    """

    import argparse
    from time import sleep
    if sys.version_info < (3, 0):
        print("Please use python version 3")
        return

    parser = argparse.ArgumentParser(add_help=True,
                                     description='Epos controller')
    parser.add_argument('--channel', '-c', action='store', default='can0',
                        type=str, help='Can channel to be used', dest='channel')
    parser.add_argument('--bus', '-b', action='store',
                        default='socketcan', type=str, help='Bus type', dest='bus')
    parser.add_argument('--rate', '-r', action='store', default=None,
                        type=int, help='bitrate, if applicable', dest='bitrate')
    parser.add_argument('--nodeID', action='store', default=1, type=int,
                        help='Node ID [ must be between 1- 127]', dest='nodeID')
    parser.add_argument('--objDict', action='store', default=None,
                        type=str, help='Object dictionary file', dest='objDict')
    args = parser.parse_args()

    # set up logging to file - see previous section for more details
    logging.basicConfig(level=logging.INFO,
                        format='[%(asctime)s.%(msecs)03d] [%(name)-20s]: %(levelname)-8s %(message)s',
                        datefmt='%d-%m-%Y %H:%M:%S',
                        filename='mqtt_controller.log',
                        filemode='w')
    # ---------------------------------------------------------------------------
    # define a Handler which writes INFO messages or higher in console
    # ---------------------------------------------------------------------------
    console = logging.StreamHandler()
    console.setLevel(logging.INFO)
    # set a format which is simpler for console use
    formatter = logging.Formatter('%(name)-20s: %(levelname)-8s %(message)s')
    # tell the handler to use this format
    console.setFormatter(formatter)
    # add the handler to the root logger
    logging.getLogger('').addHandler(console)

    # event flag to exit
    exit_flag = threading.Event()

    # TODO: in order to be able to use EPOS with the SINAMCIS, network must be shared
    # For now, create the network from scratch
    network = canopen.Network()
    try:
        network.connect(channel=args.channel, bustype=args.bus)
    except Exception as e:
        logging.info('Exception caught:{0}'.format(str(e)))
    # instantiate object
    epos = Epos_controller(_network=network)
    # declare threads
    epos_thread = threading.Thread(name="EPOS", target=epos.start_calibration,
                                   kwargs={'exit_flag': exit_flag})

    if not (epos.begin(args.nodeID, object_dictionary=args.objDict)):
        logging.info('Failed to begin connection with EPOS device')
        logging.info('Exiting now')
        return
    # emcy messages handles
    epos.node.emcy.add_callback(epos.emcy_error_print)
    # --------------------------------------------------------------------------
    # change default values for canopen sdo settings
    # --------------------------------------------------------------------------
    epos.node.sdo.MAX_RETRIES = 2
    epos.node.sdo.PAUSE_BEFORE_SEND = 0.01
    epos.node.sdo.RESPONSE_TIMEOUT = 0.02
    # -------------------------------------------------------------------------
    # test connection
    # --------------------------------------------------------------------------
    num_fails = 0
    _, success = epos.read_statusword()
    while not success and num_fails < 5:
        num_fails = num_fails + 1
        sleep(0.1)
        _, success = epos.read_statusword()
    # any success?
    if num_fails is 5:
        logging.info('Failed to contact EPOS... is it connected? Exiting')
        return
    # --------------------------------------------------------------------------
    # change PID settings
    # --------------------------------------------------------------------------
    # default values were 52, 1, 15
    # last used values 54, 1, 3
    epos.set_position_control_parameters(pGain=250, iGain=1, dGain=50)
    # show current Position control parameters
    epos.print_position_control_parameters()

    try:
        epos_thread.start()
        print("Please move steering wheel to extreme positions to calibrate...")
        input("Press Enter when done...\n")
    except KeyboardInterrupt as e:
        exit_flag.set()
        epos_thread.join()
        logging.warning('[Main] Got execption {0}... exiting now'.format(e))
        return

    exit_flag.set()
    epos_thread.join()
    if epos.calibrated == -1:
        logging.info("[Main] Failed to perform calibration")
        return
    if epos.calibrated == 0:
        logging.info("[Main] Calibration not yet done")
        return
    # reset event()
    exit_flag.clear()
    # create software position limits?
    # TODO: define max and min

    print("---------------------------------------------")
    print("Max Value: {0}\nMin Value: {1}\nZero Ref: {2}".format(
        epos.maxValue, epos.minValue, epos.zeroRef))
    print("---------------------------------------------")
    print("Moving into Zero Ref position....")
    epos.move_to_position(epos.zeroRef)
    print('Done!')
    print("---------------------------------------------")

    # ---------------------------------------------------------------------------
    # Menu definitions
    # ---------------------------------------------------------------------------
    mainMenuSaveQC = Button("Save qc to file", 1)
    mainMenuReadQC = Button("Follow qc from file", 2)
    mainMenuMove = Button("Move to position", 3)
    mainMenuShowConfig = Button("Show config", 4)
    mainMenuQuit = Button("Quit", 0)
    mainMenuButtons = [mainMenuSaveQC,
                       mainMenuReadQC, mainMenuMove, mainMenuShowConfig, mainMenuQuit]
    mainMenu = Menu("Main menu", mainMenuButtons)
    stopCycle = False
    try:
        while not stopCycle:
            val = mainMenu.display()
            if val is not None:
                if val is 0:
                    # exit program
                    stopCycle = True
                elif val is 1:
                    # save qc to a file to be used later
                    epos_thread = threading.Thread(name="Save QC",
                                                   target=epos.save_to_file,
                                                   kwargs={'exit_flag': exit_flag})
                    epos_thread.start()
                    print("Recording to file.")
                    input("Press Enter when done...\n")
                    exit_flag.set()
                    epos_thread.join()
                elif val is 2:
                    # get latest file in data dir
                    directory = pathlib.Path('./data/')
                    _, file_path = max((f.stat().st_mtime, f)
                                       for f in directory.iterdir())
                    epos.read_from_file(str(file_path), use_angle=True)
                elif val is 3:
                    try:
                        x = int(input("Enter desired position [qc]: "))
                        print(
                            '-----------------------------------------------------------')
                        print('Moving to position {0:+16,}'.format(x))
                        epos.move_to_position(x)
                        print('done')
                        print(
                            '-----------------------------------------------------------')
                        # shutdown
                        if not epos.change_state('shutdown'):
                            logging.info(
                                '[Main] Failed to change Epos state to shutdown')
                    except KeyboardInterrupt as e:
                        logging.info(
                            '[Main] Got exception {0}... exiting now'.format(e))
                elif val is 4:
                    print("Show configurations:")
                    epos.print_position_control_parameters()
                    epos.print_motor_config()
                    epos.print_sensor_config()
                    input("Press any key to continue...\n")
                else:
                    pass

    except KeyboardInterrupt as e:
        logging.info('[Main] Got exception {0}... exiting now'.format(e))
    finally:
        exit_flag.set()  # in case any thread is still working
        epos.disconnect()
    return


if __name__ == '__main__':
    main()
