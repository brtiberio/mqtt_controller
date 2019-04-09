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
import signal
import time
import numpy as np
import canopen
import pathlib
import csv
# import queue
import paho.mqtt.client as mqtt

from EPOS_Canopen.epos import Epos
from Sinamics_Canopen.sinamics import SINAMICS

import pydevd
pydevd.settrace('192.168.31.124', port=8000, stdoutToServer=True, stderrToServer=True)

# ----------------------------------------------------------------------------------------------------------------------
# mqtt topics to be used
# ----------------------------------------------------------------------------------------------------------------------
protocol = mqtt.MQTTv311

# General topics
general_topics = {'canopen': 'VIENA/General/canopen',  # canopen status
                  'rpi':     'VIENA/General/rpi',      # rpi client connected
                  'log':     'VIENA/General/log'                 # logger topic
                  }
# SINAMICS MQTT Topics
sinamics_topics = {'connected': 'VIENA/SINAMICS/connected',  # inverter connected status
                   'velocity': 'VIENA/SINAMICS/velocity',  # estimated velocity
                   'state_read': 'VIENA/SINAMICS/state/read',  # state from inverter to others
                   'state_write': 'VIENA/SINAMICS/state/write',  # state from others to inverter
                   'EMCY': 'VIENA/SINAMICS/EMCY',  # print emergency messages
                   'target_velocity_read': 'VIENA/SINAMICS/target_velocity/read',  # target velocity read
                   'target_velocity_write': 'VIENA/SINAMICS/target_velocity/write',  # target velocity write
                   }
# epos_topics = {}

# ----------------------------------------------------------------------------------------------------------------------
# Creation of logger handler to send log messages over mqtt
# ----------------------------------------------------------------------------------------------------------------------

class MQTTHandler(logging.Handler):
    """
    A handler class which writes logging records, appropriately formatted,
    to a MQTT server to a topic.
    """

    def __init__(self, client, topic, qos=0, retain=False):
        logging.Handler.__init__(self)
        self.topic = topic
        self.qos = qos
        self.retain = retain
        self.client = client

    def emit(self, record):
        """
        Publish a formatted logging record to a broker.
        """
        msg = self.format(record)
        # todo check if connected
        self.client.publish(self.topic, payload=msg,
                            qos=self.qos, retain=self.retain)


# ----------------------------------------------------------------------------------------------------------------------
# Redefined class for Epos controller to add additional functionalities
# ----------------------------------------------------------------------------------------------------------------------
class EposController(Epos):
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

    def save_to_file(self, filename=None, exit_flag=None):
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
            exit_flag: threading event flag to signal exit.
        """
        # check if inputs were supplied
        if not exit_flag:
            self.log_info('Error: exit_flag must be supplied')
            return
        # make sure is clear.
        if exit_flag.isSet():
            exit_flag.clear()
        # ----------------------------------------------------------------------
        # Confirm epos is in a suitable state for free movement
        # ----------------------------------------------------------------------
        state_id = self.check_state()
        # failed to get state?
        if state_id is -1:
            self.log_info('Error: Unknown state')
            return
        # If epos is not in disable operation at least,
        # motor is expected to be blocked
        if state_id > 4:
            self.log_info('Not a proper operation mode: {0}'.format(
                self.state[state_id]))
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
        while not exit_flag.isSet():
            current_value, ok = self.read_position_value()
            t_out = time.monotonic() - t0
            if not ok:
                self.log_info('Failed to request current position')
                num_fails = num_fails + 1
            else:
                writer.writerow({'time': t_out, 'position': current_value,
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
                    # if is not the first position but t_out is not yet t_target
                    # sleep
                    # skip to next step?
                    while True:
                        t_out = time.monotonic() - t0
                        # is time to send new values?
                        if t_out > t_target:
                            # time to update
                            if not self.set_position_mode_setting(position):
                                num_fails = num_fails + 1
                            break
                        # if we are not sending new targets, request current value to see the error
                        if t_out - last_read > 0.051:
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
                                        t_out, position, aux, ref_error))
                                    if not self.change_state('shutdown'):
                                        self.log_info(
                                            'Failed to change Epos state to shutdown')
                                    return
                                # for debug print every time on each cycle
                                print('time={0:+08.3f}\tIn={1:+05}\tOut={2:+05}\tError={3:+05}'.format(
                                    t_out, position, aux, ref_error))
                            last_read = t_out
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
            boolean: True if all went as expected or false otherwise

        .. [1] Li, Huaizhong & M Gong, Z & Lin, Wei & Lippa, T. (2007). Motion profile planning for reduced jerk and vibration residuals. 10.13140/2.1.4211.2647.
        """
        # constants
        # t_max = 1.7 seems to be the limit before oscillations.
        t_max = 0.2  # max period for 1 rotation;
        # 1 rev = 3600*4 [qc]
        countsPerRev = 3600 * 4
        #
        # 1Hz = 60rpm = 360degrees/s
        #
        # 360 degrees = sensor resolution * 4
        #
        # this yields: 1Hz = (sensor resolution * 4)/s
        #
        # Fmax = 1 / t_max;
        #
        # max_speed = 60 rpm/t_max [rpm]=
        #          = 360degrees/t_max [degrees/s]=
        #          = (sensor resolution *4)/t_max [qc/s]

        max_speed = countsPerRev / t_max  # degrees per sec

        # max acceleration must be experimental obtained.
        # reduced and fixed.
        max_acceleration = 6000.0  # [qc]/s^2

        # maximum interval for both the acceleration  and deceleration phase are:
        t1_max = 2.0 * max_speed / max_acceleration  # type: float

        # the max distance covered by these two phase (assuming acceleration equal
        # deceleration) is 2* 1/4 * Amax * t1_max^2 = 1/2 * Amax * t1_max^2 = 2Vmax^2/Amax
        max_l13 = 2.0 * max_speed ** 2 / max_acceleration  # type: float

        # max error in quadrature counters
        max_error = 7500
        # num_fails = 0
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

        p_start, ok = self.read_position_value()
        num_fails = 0
        if not ok:
            self.log_info('Failed to request current position')
            while num_fails < 5 and not ok:
                p_start, ok = self.read_position_value()
                if not ok:
                    num_fails = num_fails + 1
            if num_fails == 5:
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
        l = abs(pos_final - p_start)
        if l is 0:
            # already in final point
            return True
        # do we need  a constant velocity phase?
        if l > max_l13:
            t2 = 2.0 * (l - max_l13) / (max_acceleration * t1_max)
            t1 = t1_max
            t3 = t1_max
        else:
            t1 = np.sqrt(2 * l / max_acceleration)
            t2 = 0.0
            t3 = t1

        # time constants
        t1 = t1
        t2 = t2 + t1
        t3 = t3 + t2  # final time

        # allocate vars
        in_var = np.array([], dtype='int32')
        out_var = np.array([], dtype='int32')
        tin = np.array([], dtype='int32')
        tout = np.array([], dtype='int32')
        ref_error = np.array([], dtype='int32')

        # determine the sign of movement
        move_up_or_down = np.sign(pos_final - p_start)
        flag = True
        pi = np.pi
        cos = np.cos
        time.sleep(0.01)
        # choose monotonic for precision
        t0 = time.monotonic()
        num_fails = 0
        while flag and not self.errorDetected:
            # request current time
            tin = np.append(tin, [time.monotonic() - t0])
            # time to exit?
            if tin[-1] > t3:
                flag = False
                in_var = np.append(in_var, [pos_final])
                self.set_position_mode_setting(pos_final)
                # reading a position takes time, as so, it should be enough
                # for it reaches end value since steps are expected to be
                # small
                aux, ok = self.read_position_value()
                if not ok:
                    self.log_info('Failed to request current position')
                    num_fails = num_fails + 1
                else:
                    out_var = np.append(out_var, [aux])
                    tout = np.append(tout, [time.monotonic() - t0])
                    ref_error = np.append(ref_error, [in_var[-1] - out_var[-1]])
            # not finished
            else:
                # get reference position for that time
                if tin[-1] <= t1:
                    aux = p_start + \
                          move_up_or_down * max_acceleration / 2.0 * (t1 / (2.0 * pi)) ** 2 * \
                          (1 / 2.0 * (2.0 * pi / t1 *
                                      tin[-1]) ** 2 - (1.0 - cos(2.0 / t1 * pi * tin[-1])))
                else:
                    if (t2 > 0 and tin[-1] > t1 and tin[-1] <= t2):
                        aux = p_start + \
                              move_up_or_down * \
                              (1 / 4.0 * max_acceleration * t1 ** 2 + 1 /
                               2.0 * max_acceleration * t1 * (tin[-1] - t1))
                    else:
                        aux = p_start + \
                              move_up_or_down * (1 / 4.0 * max_acceleration * t1 ** 2
                                                 + 1 / 2.0 * max_acceleration * t1 * t2 +
                                                 max_acceleration / 2.0 *
                                                 (t1 / (2.0 * pi)) ** 2
                                                 * ((2.0 * pi) ** 2 * (tin[-1] - t2) / t1 - 1 / 2.0 * (2.0 * pi / t1
                                                                                                       * (tin[
                                                                                                              -1] - t2)) ** 2 + (
                                                            1.0 - cos(2.0 * pi / t1 * (tin[-1] - t2)))))
                aux = round(aux)
                # append to array and send to device
                in_var = np.append(in_var, [aux])
                ok = self.set_position_mode_setting(np.int32(in_var[-1]).item())
                if not ok:
                    self.log_info('Failed to set target position')
                    num_fails = num_fails + 1
                aux, ok = self.read_position_value()
                if not ok:
                    self.log_info('Failed to request current position')
                    num_fails = num_fails + 1
                else:
                    out_var = np.append(out_var, [aux])
                    tout = np.append(tout, [time.monotonic() - t0])
                    ref_error = np.append(ref_error, [in_var[-1] - out_var[-1]])
                    if abs(ref_error[-1]) > max_error:
                        self.change_state('shutdown')
                        self.log_info(
                            'Something seems wrong, error is growing to mutch!!!')
                        return False
            # require sleep?
            time.sleep(0.005)
        self.log_info('Finished with {0} fails'.format(num_fails))
        return True

# ----------------------------------------------------------------------------------------------------------------------
# Redefined class for inverter controller to add additional functionalities
# ----------------------------------------------------------------------------------------------------------------------


class SinamicsController(SINAMICS):

    def emcy_error_print(self, emcy_error):
        """ Override default emcy error print to include the MQTT publisher.
        """
        if emcy_error.code is 0:
            return
        else:
            fault_number = int.from_bytes(emcy_error.data[0:2], 'little')
            drive_object_number = int(emcy_error.data[2])
            if fault_number in self.sinamics_fault_number:
                description = self.sinamics_fault_number[fault_number]
            else:
                description = "unknown"
            self.log_info('Got an EMCY message {0}'.format(emcy_error))
            self.log_info('Sinamics error number: {0} with \'{1}\' in drive unit {2}'.format(fault_number,
                                                                                             description,
                                                                                             drive_object_number))
            if client.connected:
                client.publish(sinamics_topics['EMCY'], payload=description, qos=1)
            return

    def print_velocity(self, message):
        """Print velocity value received from PDO

        Args:
            message: message received in PDO
        """
        self.log_debug('{0} received'.format(message.name))
        for var in message:
            self.log_debug('{0} = {1:06X}'.format(var.name, var.raw))
            if var.index == 0x6041:
                if not client.connected:
                    pass
                else:
                    state = self.check_state(var.raw)
                    client.publish(sinamics_topics['state_read'], payload=state.to_bytes(1, 'little'))
            if var.index == 0x606C:
                if not client.connected:
                    self.log_info('{0:+05d} RPM'.format(var.raw))
                else:
                    client.publish(sinamics_topics['velocity'], payload=var.raw.to_bytes(4, 'little', signed=True))


# ---------------------------------------------------------------------------
# define signal handlers for systemd signals
# ---------------------------------------------------------------------------
def signal_handler(signum, frame):
    if signum == signal.SIGINT:
        logging.info('Received signal INTERRUPT... exiting now')
    if signum == signal.SIGTERM:
        logging.info('Received signal TERM... exiting now')
    client.cleanExit()
    return


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
                                     description='MQTT controller')
    parser.add_argument('--channel', '-c', action='store', default='can0',
                        type=str, help='Can channel to be used', dest='channel')
    parser.add_argument('--bus', '-b', action='store',
                        default='socketcan', type=str, help='Bus type', dest='bus')
    parser.add_argument('--rate', '-r', action='store', default=None,
                        type=int, help='bitrate, if applicable', dest='bitrate')
    parser.add_argument('--hostname', action='store', default='localhost', type=str,
                        help='hostname for mqtt broker', dest='hostname')
    parser.add_argument('--port', '-p', action='store', default=8080, type=int,
                        help='port for mqtt broker', dest='port')
    parser.add_argument('--path', action='store', default='/mqtt', type=str,
                        help='path to be used in mqtt broker', dest='path')
    parser.add_argument('--transport', action='store', default='websockets', type=str,
                        help='transport layer used in mqtt broker', dest='transport')
    parser.add_argument("--log-level", action="store", type=str,
                        dest="logLevel", default='info',
                        help='Log level to be used. See logging module for more info',
                        choices=['critical', 'error', 'warning', 'info', 'debug'])

    args = parser.parse_args()
    log_level = {'error': logging.ERROR,
                 'debug': logging.DEBUG,
                 'info': logging.INFO,
                 'warning': logging.WARNING,
                 'critical': logging.CRITICAL
                 }
    # ---------------------------------------------------------------------------
    # Important constants and definitions to be used
    # ---------------------------------------------------------------------------
    epos_node_id = 1
    sinamics_node_id = 2
    sinamics_obj_dict = 'Sinamics_Canopen/sinamics_s120.eds'

    # mqtt constants
    hostname = args.hostname
    port = args.port
    transport = args.transport
    exit_flag = threading.Event()  # event flag to exit
    # ---------------------------------------------------------------------------
    # set up logging to file to used debug level saved to disk
    # ---------------------------------------------------------------------------
    logging.basicConfig(level=log_level[args.logLevel],
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
    # ---------------------------------------------------------------------------
    # Defines of callback functions
    # ---------------------------------------------------------------------------

    def on_message(self, userdata, message):
        if message.topic == general_topics[2]:
            logging.info('Received message: "' + str(message.payload.decode('UTF-8')) + '" on topic '
                         + message.topic + ' with QoS ' + str(message.qos))
        else:
            logging.info("Received message :" + str(message.payload) + " on topic "
                         + message.topic + " with QoS " + str(message.qos))

    def on_connect(client, userdata, flags, rc):
        if rc == 0:
            client.connected = True
            # successfully connected
            (rc, _) = client.publish(general_topics['rpi'], payload=True.to_bytes(1, 'little'),
                                   qos=2, retain=True)
            if rc is mqtt.MQTT_ERR_SUCCESS:
                # now add mqttLog to root logger to enable it
                logging.getLogger('').addHandler(mqtt_logger)

                # TODO subscribe to other topics
            else:
                logging.info('Unexpected result on publish: rc={0}'.format(rc))
        else:
            logging.info("Failed to connect to server")
        return

    def on_disconnect(self, userdata, rc):
        if rc != 0:
            logging.info("Unexpected MQTT disconnection. Will auto-reconnect")


    def clean_exit():
        """Handle exiting request

        Before exiting, send a message to mqtt broker to correctly signal the
        disconnection.
        The function must be appended as method to mqtt client object.
        """
        # tell we are disconnected on canopen topic
        (rc, _) = client.publish(general_topics['canopen'], payload=False.to_bytes(1, 'little'),
                                 qos=2, retain=True)
        if rc is not mqtt.MQTT_ERR_SUCCESS:
            logging.info('Failed to publish on exit: {0}'.format(general_topics['canopen']))

        # tell we are disconnected on rpi topic
        (rc, _) = client.publish(general_topics['rpi'], payload=False.to_bytes(1, 'little'),
                                 qos=2, retain=True)
        if rc is not mqtt.MQTT_ERR_SUCCESS:
            logging.info('Failed to publish on exit: {0}'.format(general_topics['rpi']))
        sleep(1)
        # wait for all messages are published before disconnect
        while len(client._out_messages):
            sleep(0.01)
        client.disconnect()
        return
    # ---------------------------------------------------------------------------
    # end of callback defines
    # ---------------------------------------------------------------------------
    # ---------------------------------------------------------------------------
    # mqtt client configure
    # ---------------------------------------------------------------------------
    global client
    client = mqtt.Client(protocol=protocol, transport=transport)
    # in case of lost connection, tell other we are dead.
    client.will_set(general_topics['rpi'], payload=False.to_bytes(1, 'little'),
                    qos=2, retain=True)
    client.on_connect = on_connect
    client.on_message = on_message
    client.on_disconnect = on_disconnect
    client.cleanExit = clean_exit
    client.connected = False
    # ---------------------------------------------------------------------------
    # setup mqtt_controller logger to transmit logging messages via mqtt but do
    # not activate it. Activate only when sucessfull connected to broker.
    # ---------------------------------------------------------------------------
    mqtt_logger = MQTTHandler(client, general_topics['log'])
    # save all levels
    mqtt_logger.setLevel(logging.INFO)
    mqtt_logger.setFormatter(logging.Formatter(fmt='[%(asctime)s.%(msecs)03d] [%(name)-20s]: %(levelname)-8s %(message)s',
                                               datefmt='%d-%m-%Y %H:%M:%S'))
    # ---------------------------------------------------------------------------
    no_faults = True
    try:
        client.connect(hostname, port=port)
        client.loop_start()
    except Exception as e:
        logging.info('Connection failed: {0}'.format(str(e)))
        no_faults = False
    finally:
        if not no_faults:
            client.loop_stop(force=True)
            logging.info('Failed to connect to broker...Exiting')
            return
    # ---------------------------------------------------------------------------
    # TODO: in order to be able to use EPOS with the SINAMICS, network must be shared
    # For now, create the network from scratch
    # ---------------------------------------------------------------------------
    network = canopen.Network()
    no_faults = True
    sleep(1)
    try:
        network.connect(channel=args.channel, bustype=args.bus)
        # if no exception, connection is made. Signal in mqtt.
        client.publish(general_topics['canopen'], payload=True.to_bytes(1, 'little'),
                       qos=2, retain=True)
    except Exception as e:
        logging.info('Exception caught:{0}'.format(str(e)))
        no_faults = False
    finally:
        if not no_faults:
            logging.info('Failed to connect to can network...Exiting')
            client.cleanExit()
            client.loop_stop(force=True)
            return
    # instantiate Sinamics inverter object using external network
    inverter = SinamicsController(_network=network)

    if not (inverter.begin(sinamics_node_id, object_dictionary=sinamics_obj_dict)):
        logging.info('Failed to begin connection with Sinamics device')
        logging.info('Exiting now')
        client.publish(sinamics_topics['connected'], payload=False.to_bytes(1, 'little'), qos=2, retain=True)
        client.cleanExit()
        client.loop_stop(force=True)
        return
    # if successfully connected, publish it to sinamics connected topic
    client.publish(sinamics_topics['connected'], payload=True.to_bytes(1, 'little'), qos=2, retain=True)

    # emcy messages handles
    inverter.node.emcy.add_callback(inverter.emcy_error_print)
    # --------------------------------------------------------------------------
    # change default values for canopen sdo settings
    # --------------------------------------------------------------------------
    inverter.node.sdo.MAX_RETRIES = 2
    # inverter.node.sdo.PAUSE_BEFORE_SEND = 0.01
    # inverter.node.sdo.RESPONSE_TIMEOUT = 0.02
    # -------------------------------------------------------------------------
    # test connection
    # --------------------------------------------------------------------------
    num_fails = 0
    state = inverter.check_state()
    while not state and num_fails < 5:
        num_fails = num_fails + 1
        sleep(0.1)
        state = inverter.check_state()
    # any success?
    if num_fails is 5:
        logging.info('Failed to contact inverter... is it connected? Exiting')
        client.publish(sinamics_topics['connected'], payload=False.to_bytes(1, 'little'), qos=2, retain=True)
        client.cleanExit()
        client.loop_stop(force=True)
        return
    # send current state to mqtt
    client.publish(sinamics_topics['state_read'], payload=state.to_bytes(1, 'little'), qos=1, retain=True)
    # --------------------------------------------------------------------------
    # configure pdo objects
    # --------------------------------------------------------------------------
    inverter.node.pdo.read()

    inverter.node.nmt.state = 'PRE-OPERATIONAL'
    inverter.node.pdo.tx[1].clear()
    inverter.node.pdo.tx[2].clear()
    inverter.node.pdo.tx[3].clear()
    inverter.node.pdo.tx[4].clear()

    inverter.node.pdo.rx[1].clear()
    inverter.node.pdo.rx[2].clear()
    inverter.node.pdo.rx[3].clear()
    inverter.node.pdo.rx[4].clear()

    # Do some changes to TxPDO2
    inverter.node.pdo.tx[2].clear()
    inverter.node.pdo.tx[2].add_variable(0x6041, 0, 16)
    inverter.node.pdo.tx[2].add_variable(0x606C, 0, 32)
    inverter.node.pdo.tx[2].enabled = True
    # inverter.node.pdo.tx[2].event_timer = 2000
    inverter.node.pdo.tx[2].trans_type = 254

    inverter.change_state('fault reset')
    sleep(0.1)
    # Save parameters to device
    inverter.node.pdo.tx[2].save()

    # Add callback for message reception
    inverter.node.pdo.tx[2].add_callback(inverter.print_velocity)
    # --------------------------------------------------------------------------
    # Set back into operational mode
    inverter.node.nmt.state = 'OPERATIONAL'
    # TODO change State is failing. to be checked
    sleep(0.1)
    inverter.change_state('shutdown')
    sleep(0.1)
    inverter.change_state('switch on')
    sleep(0.1)
    inverter.change_state('enable operation')
    inverter.print_current_smoothed()

    try:
        print("Ctrl+C to exit... ")
        while True:
            velocity = int(input('Set your velocity...'))
            if velocity is None:
                pass
            else:
                print('Setting velocity to {0}'.format(velocity))
                inverter.set_target_velocity(velocity)
            sleep(3)
            inverter.print_current_smoothed()
    except KeyboardInterrupt as e:
        logging.info('[Main] Got exception {0}... exiting now'.format(e))
    finally:
        exit_flag.set()  # in case any thread is still working
        client.publish(general_topics['canopen'], payload='Disconnected',
                       qos=2, retain=True)
        client.cleanExit()
        client.loop_stop(force=True)
    return


if __name__ == '__main__':
    main()
