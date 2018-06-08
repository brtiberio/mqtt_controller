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
import os
import pathlib
import csv


# import queue
# import paho.mqtt.client as mqtt


from EPOS_Canopen.epos import Epos

# shortcut for clear console


def clear():
    os.system('cls' if os.name == 'nt' else 'clear')


class Menu(object):

    """Base class for the menu"""

    def __init__(self, name, buttons):
        # Initialize values
        self.name = name
        self.buttons = buttons
        self.exitFlag = False

    def clear(self):
        os.system('cls' if os.name == 'nt' else 'clear')

    def display(self):
        """Displaying the menu alongside the navigation elements"""

        self.clear()
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

    maxFollowingError = 5000
    minValue = 0
    maxValue = 0
    zeroRef = 0
    calibrated = 0
    QC_TO_DELTA = -7.501E-4
    DELTA_TO_QC = 1.0/QC_TO_DELTA
    maxAngle = 28
    minAngle = -maxAngle
    dataDir = "./data/"


    def emcyErrorPrint(self, EmcyError):
        '''Print any EMCY Error Received on CAN BUS
        '''
        logging.info('[{0}] Got an EMCY message: {1}'.format(
            sys._getframe().f_code.co_name, EmcyError))
        return

    def getQcPosition(self, delta):
        ''' Converts angle of wheels to qc

        Given the desired angle of wheels, in degrees of the bicicle model of car,
        convert the requested value to qc position of steering wheel using the
        calibration performed at beginning.

        Args:
            delta: desired angle of wheels in degrees.
        Returns:
            int: a rounded integer with qc position estimated or None if not possible
        '''
        if not self.calibrated:
            logging.info('[{0}] Device is not yet calibrated'.format(
                sys._getframe().f_code.co_name))
            return None
        if delta > self.maxAngle:
            logging.info('[{0}] Angle exceeds limits: maxAngle: {1}\t requested: {2}'.format(
                sys._getframe().f_code.co_name,
                self.maxAngle,
                delta))
            return None
        if delta < self.minAngle:
            logging.info('[{0}] Angle exceeds limits: minAngle: {1}\t requested: {2}'.format(
                sys._getframe().f_code.co_name,
                self.minAngle,
                delta))
            return None
        # perform calculations y = mx + b
        val = delta*self.DELTA_TO_QC + self.zeroRef
        val = round(val)
        return val

    def getDeltaAngle(self, qc):
        ''' Converts qc of steering wheel to angle of wheels

        Given the desired qc steering position, in degrees of the bicicle model of car,
        convert the requested value to angle in degrees.

        Args:
            qc: an int with desired qc position of steering wheel.
        Returns:
            double: estimated angle of wheels in degrees or None if not possible
        '''
        if not self.calibrated:
            logging.info('[{0}] Device is not yet calibrated'.format(
                sys._getframe().f_code.co_name))
            return None

        # perform calculations y = mx + b and solve to x
        delta = (qc-self.zeroRef)*self.QC_TO_DELTA
        return delta

    def saveToFile(self, filename=None, exitFlag=None):
        '''Record qc positions into a csv file

        The following fields will be recorded

        +-------+----------+
        | time  | position |
        +-------+----------+
        | t1    | p1       |
        +-------+----------+
        | ...   | ...      |
        +-------+----------+
        | tN    | pN       |
        +-------+----------+

        An adicional file with same name but with ext TXT will have the current
        calibration parameters

         * minValue
         * maxValue
         * zeroRef

        If filename is not supplied or already used, the current asctime() will
        be used as filename.

        Args:
            filename: name of the file to save the data
        '''
        # check if inputs were supplied
        if not exitFlag:
            logging.info('[{0}] Error: exitFlag must be supplied'.format(
                sys._getframe().f_code.co_name))
            return
        # -----------------------------------------------------------------------
        # Confirm epos is in a suitable state for free movement
        # -----------------------------------------------------------------------
        stateID = self.checkEposState()
        # failed to get state?
        if stateID is -1:
            logging.info('[{0}] Error: Unknown state'.format(
                sys._getframe().f_code.co_name))
            return
        # If epos is not in disable operation at least, motor is expected to be blocked
        if stateID > 4:
            logging.info('[{0}] Not a proper operation mode: {1}'.format(
                sys._getframe().f_code.co_name,
                self.state[stateID]))
            logging.info('[{0}] Please change operation mode'.format(
                sys._getframe().f_code.co_name))
            return
        # all ok, proceed
        if not filename:
            filename = time.asctime()

        # make dir if not already made
        pathlib.Path(self.dataDir).mkdir(parents=True, exist_ok=True)
        my_file = pathlib.Path(self.dataDir+filename+'.csv')
        if not my_file.exists():
            # create a new name
            filename = time.asctime()

        # open the parameters file first
        my_file = open(self.dataDir + filename + '.txt', 'w')
        print("minValue = {0}\nmaxValue = {1}\nzeroRef = {2}".format(self.minValue,
                                                                     self.maxValue, self.zeroRef), file=my_file, flush=True)
        my_file.close()

        # open the csv file
        my_file = open(self.dataDir + filename + '.csv', 'w')
        writer = csv.DictWriter(my_file, fieldnames=['time', 'position'])
        writer.writeheader()
        # -----------------------------------------------------------------------
        # start requesting for positions of sensor
        # -----------------------------------------------------------------------
        # get current time
        t0 = time.monotonic()
        while(exitFlag.isSet() == False):
            currentValue, OK = self.readPositionValue()
            tOut = time.monotonic()-t0
            if not OK:
                logging.info('({0}) Failed to request current position'.format(
                    sys._getframe().f_code.co_name))
                return
            writer.writerow({'time': tOut, 'position': currentValue})
            # sleep?
            time.sleep(0.01)

        logging.info('[{0}] Finishing collecting data'.format(
            sys._getframe().f_code.co_name))
        my_file.close()

    def readFromFile(self, filename=None):
        '''Read qc positions from file and follow them

        The file must contain time and position in quadrature positions of steering
        wheel in a csv style

        +-------+----------+
        | time  | position |
        +-------+----------+
        | t1    | p1       |
        +-------+----------+
        | ...   | ...      |
        +-------+----------+
        | tN    | pN       |
        +-------+----------+

        Args:
            filename: csv file to be read.
        '''
        # get current state of epos
        state = self.checkEposState()
        if state is -1:
            logging.info('[Epos:{0}] Error: Unknown state'.format(
                sys._getframe().f_code.co_name))
            return

        if state is 11:
            # perform fault reset
            if not self.changeEposState('fault reset'):
                logging.info('[Epos:{0}] Error: Failed to change state to fault reset'.format(
                    sys._getframe().f_code.co_name))
                return

        # shutdown
        if not self.changeEposState('shutdown'):
            logging.info('[Epos:{0}] Failed to change Epos state to shutdown'.format(
                    sys._getframe().f_code.co_name))
            return
        # switch on
        if not self.changeEposState('switch on'):
            logging.info('[Epos:{0}] Failed to change Epos state to switch on'.format(
                    sys._getframe().f_code.co_name))
            return
        if not self.changeEposState('enable operation'):
            logging.info('[Epos:{0}] Failed to change Epos state to enable operation'.format(
                    sys._getframe().f_code.co_name))
            return

        # check if file exist
        my_file = pathlib.Path(self.dataDir+filename+'.csv')
        if not my_file.exists():
            logging.info('[Epos:{0}] File does not exist: {1}'.format(
                    sys._getframe().f_code.co_name,
                    my_file))
            return

        # open csv file and read all values.
        with open(my_file) as csvfile:
            reader = csv.DictReader(csvfile, delimiter=',')
            data = {}
            for row in reader:
                for header, value in row.items():
                    try:
                        data[header].append(int(value))
                    except KeyError:
                        data[header] = [value]
                    except ValueError:
                        data[header].append(float(value))
        I = 0
        maxI = len(data['time'])
        t0 = time.monotonic()
        while(I < maxI):
            tOut = time.monotonic()-t0
            # skip to next step?
            if tOut > data['time'][I]:
                I += 1
                updateFlag = True
            else:
                # send data only once
                if (updateFlag):
                    updateFlag = False
                    # get new reference position.
                    self.setPositionModeSetting(data['position'][I])
                else:
                    # if not time to update new ref, request current position
                    aux, OK = self.readPositionValue()
                    if not OK:
                        logging.info('[Epos:{0}] Failed to request current position'.format(
                            sys._getframe().f_code.co_name))
                        return
                    # does error have grown to much?
                    if abs(data['position'][I]-aux) > self.maxFollowingError:
                        logging.info('[Epos:{0}] Error is growing to much. Something seems wrong'.format(
                            sys._getframe().f_code.co_name))
                        if not self.changeEposState('shutdown'):
                            logging.info('[Epos:{0}] Failed to change Epos state to shutdown'.format(
                            sys._getframe().f_code.co_name))
                        return
                    # use sleep?
                    time.sleep(0.005)
        # all done
        logging.info('[Epos:{0}] All done: Time to process all vars was {1} seconds'.format(
            sys._getframe().f_code.co_name, time.monotonic()-t0))
        if not self.changeEposState('shutdown'):
            logging.info('[Epos:{0}] Failed to change Epos state to shutdown'.format(
                            sys._getframe().f_code.co_name))
        return


    def startCalibration(self, exitFlag=None):
        '''Perform steering wheel calibration

        This function is expected to be run on a thread in order to find the limits
        of the steering wheel position and find the expected value of the zero angle
        of wheels.

        Args:
            exitFlag: threading.Event() to signal the finish of acquisition

        '''
        # check if inputs were supplied
        if not exitFlag:
            logging.info('[{0}] Error: check arguments supplied'.format(
                sys._getframe().f_code.co_name))
            return
        stateID = self.checkEposState()
        # -----------------------------------------------------------------------
        # Confirm epos is in a suitable state for free movement
        # -----------------------------------------------------------------------
        # failed to get state?
        if stateID is -1:
            logging.info('[{0}] Error: Unknown state'.format(
                sys._getframe().f_code.co_name))
            return
        # If epos is not in disable operation at least, motor is expected to be blocked
        if stateID > 4:
            logging.info('[{0}] Not a proper operation mode: {1}'.format(
                sys._getframe().f_code.co_name,
                self.state[stateID]))
            logging.info('[{0}] Please change operation mode'.format(
                sys._getframe().f_code.co_name))
            return

        maxValue = 0
        minValue = 0
        # -----------------------------------------------------------------------
        # start requesting for positions of sensor
        # -----------------------------------------------------------------------
        while(exitFlag.isSet() == False):
            currentValue, OK = self.readPositionValue()
            if not OK:
                logging.info('({0}) Failed to request current position'.format(
                    sys._getframe().f_code.co_name))
                self.minValue = None
                self.maxValue = None
                self.calibrated = -1
                return
            if currentValue > maxValue:
                maxValue = currentValue
            if currentValue < minValue:
                minValue = currentValue
            # sleep?
            time.sleep(0.01)

        logging.info('[{0}] Finishing calibration routine'.format(
            sys._getframe().f_code.co_name))
        self.minValue = minValue
        self.maxValue = maxValue
        self.zeroRef = round((maxValue-minValue)/2.0)
        self.calibrated = 1
        return

    def moveToPosition(self, pFinal):
        # constants
        # Tmax = 1.7 seems to be the limit before oscillations.
        Tmax = 1.7  # max period for 1 rotation;
        # 1 rev = 3600*4 [qc]
        countsPerRev = 3600*4
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

        maxSpeed = countsPerRev/Tmax  # degrees per sec

        # max acceleration must be experimental obtained.
        # reduced and fixed.
        maxAcceleration = 6000.0  # [qc]/s^2

        # maximum interval for both the accelleration  and deceleration phase are:
        T1max = 2.0 * maxSpeed/maxAcceleration

        # the max distance covered by these two phase (assuming acceleration equal
        # deceleration) is 2* 1/4 * Amax * T1max^2 = 1/2 * Amax * T1max^2 = 2Vmax^2/Amax
        maxL13 = 2.0 * maxSpeed**2/maxAcceleration

        # max error in quadrature counters
        MAXERROR = 5000
        if not self.calibrated:
            logging.info('[{0}] Device is not yet calibrated'.format(
                sys._getframe().f_code.co_name))
            return False

        if(pFinal > self.maxValue or pFinal < self.minValue):
            logging.info('[{0}] Final position exceeds phisical limits'.format(
                sys._getframe().f_code.co_name))
            return False

        pStart, OK = self.readPositionValue()
        if not OK:
            logging.info('({0}) Failed to request current position'.format(
                sys._getframe().f_code.co_name))
            return False
        # -----------------------------------------------------------------------
        # get current state of epos and change it if necessary
        # -----------------------------------------------------------------------
        state = self.checkEposState()
        if state is -1:
            logging.info('[Epos:{0}] Error: Unknown state\n'.format(
                sys._getframe().f_code.co_name))
            return

        if state is 11:
            # perform fault reset
            ok = self.changeEposState('fault reset')
            if not ok:
                logging.info('[Epos:{0}] Error: Failed to change state to fault reset\n'.format(
                    sys._getframe().f_code.co_name))
                return

        # shutdown
        if not self.changeEposState('shutdown'):
            logging.info('Failed to change Epos state to shutdown')
            return
        # switch on
        if not self.changeEposState('switch on'):
            logging.info('Failed to change Epos state to switch on')
            return
        if not self.changeEposState('enable operation'):
            logging.info('Failed to change Epos state to enable operation')
            return
        # -----------------------------------------------------------------------
        # Find remaining constants
        # -----------------------------------------------------------------------
        # absolute of displacement
        l = abs(pFinal - pStart)
        if l is 0:
            # already in final point
            return
        # do we need  a constant velocity phase?
        if(l > maxL13):
            T2 = 2.0*(l - maxL13)/(maxAcceleration*T1max)
            T1 = T1max
            T3 = T1max
        else:
            T1 = np.sqrt(2*l/maxAcceleration)
            T2 = 0.0
            T3 = T1

        # time constanst
        t1 = T1
        t2 = T2+t1
        t3 = T3+t2  # final time

        # allocate vars
        inVar = np.array([], dtype='int32')
        outVar = np.array([], dtype='int32')
        tin = np.array([], dtype='int32')
        tout = np.array([], dtype='int32')
        ref_error = np.array([], dtype='int32')

        # determine the sign of movement
        moveUp_or_down = np.sign(pFinal-pStart)
        flag = True
        pi = np.pi
        cos = np.cos
        time.sleep(0.01)

        t0 = time.monotonic()
        while flag:
            # request current time
            tin = np.append(tin, [time.monotonic()-t0])
            # time to exit?
            if tin[-1] > t3:
                flag = False
                inVar = np.append(inVar, [pFinal])
                self.setPositionModeSetting(pFinal)
                aux, OK = self.readPositionValue()
                if not OK:
                    logging.info('({0}) Failed to request current position'.format(
                        sys._getframe().f_code.co_name))
                    return
                outVar = np.append(outVar, [aux])
                tout = np.append(tout, [time.monotonic()-t0])
                ref_error = np.append(ref_error, [inVar[-1]-outVar[-1]])
                # update plot
                # plotter.update(tin, tout, inVar, outVar, ref_error)
            # not finished
            else:
                # get reference position for that time
                if (tin[-1] <= t1):
                    aux = pStart + \
                        moveUp_or_down * maxAcceleration/2.0 * (T1/(2.0*pi))**2 * \
                        (1/2.0 * (2.0 * pi/T1 *
                                  tin[-1])**2 - (1.0-cos(2.0/T1 * pi * tin[-1])))
                else:
                    if (T2 > 0 and tin[-1] > t1 and tin[-1] <= t2):
                        aux = pStart + \
                            moveUp_or_down * \
                            (1/4.0 * maxAcceleration * T1**2 + 1 /
                             2.0 * maxAcceleration*T1 * (tin[-1]-t1))
                    else:
                        aux = pStart + \
                            moveUp_or_down * (1/4.0 * maxAcceleration * T1**2
                                              + 1/2.0 * maxAcceleration * T1*T2 +
                                              maxAcceleration/2.0 *
                                              (T1/(2.0*pi))**2
                                              * ((2.0*pi)**2 * (tin[-1]-t2)/T1 - 1/2.0*(2.0*pi/T1
                                                                                        * (tin[-1]-t2))**2 + (1.0 - cos(2.0*pi/T1*(tin[-1]-t2)))))
                aux = round(aux)
                # append to array and send to device
                inVar = np.append(inVar, [aux])
                OK = self.setPositionModeSetting(np.int32(inVar[-1]).item())
                if not OK:
                    logging.info('({0}) Failed to set target position'.format(
                        sys._getframe().f_code.co_name))
                    return
                aux, OK = self.readPositionValue()
                if not OK:
                    logging.info('({0}) Failed to request current position'.format(
                        sys._getframe().f_code.co_name))
                    return
                outVar = np.append(outVar, [aux])
                tout = np.append(tout, [time.monotonic()-t0])
                ref_error = np.append(ref_error, [inVar[-1]-outVar[-1]])
                if(abs(ref_error[-1]) > MAXERROR):
                    self.changeEposState('shutdown')
                    print('Something seems wrong, error is growing to mutch!!!')
                    return
        # plotter.update(tin, tout, inVar, outVar, ref_error)
        # require sleep?
        time.sleep(0.001)

def main():
    '''Perform steering wheel calibration.

    Ask user to turn the steering wheel to the extremes and finds the max
    '''

    import argparse
    if (sys.version_info < (3, 0)):
        print("Please use python version 3")
        return

    parser = argparse.ArgumentParser(add_help=True,
                                     description='Epos controller')
    parser.add_argument('--channel', '-c', action='store', default='can0',
                        type=str, help='Channel to be used', dest='channel')
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
                        filename='epos.log',
                        filemode='w')
    # define a Handler which writes INFO messages or higher
    console = logging.StreamHandler()
    console.setLevel(logging.INFO)
    # set a format which is simpler for console use
    formatter = logging.Formatter('%(name)-20s: %(levelname)-8s %(message)s')
    # tell the handler to use this format
    console.setFormatter(formatter)
    # add the handler to the root logger
    logging.getLogger('').addHandler(console)

    # event flag to exit
    exitFlag = threading.Event()

    # TODO: in order to be able to use EPOS with the SINAMCIS, network must be shared
    # For now, create the network from scratch
    network = canopen.Network()
    network.connect(channel=args.channel, bustype=args.bus)

    # instanciate object
    epos = Epos_controller(_network=network)
    # declare threads
    eposThread = threading.Thread(name="EPOS", target=epos.startCalibration,
                                       args=(exitFlag))

    if not (epos.begin(args.nodeID, objectDictionary=args.objDict)):
        logging.info('Failed to begin connection with EPOS device')
        logging.info('Exiting now')
        return
    # emcy messages handles
    epos.node.emcy.add_callback(epos.emcyErrorPrint)

    try:
        eposThread.start()
        print("Please move steering wheel to extreme positions to calibrate...")
        input("Press Enter when done...")
    except KeyboardInterrupt as e:
        logging.warning('Got execption {0}... exiting now'.format(e))
        return

    exitFlag.set()
    eposThread.join()
    if(epos.calibrated == -1):
        print("Failed to perform calibration")
        return
    if(epos.calibrated == 0):
        print("Calibration not yet done")
        return
    # reset event()
    exitFlag.clear()
    # create software position limits?
    # TODO: define max and min

    print("---------------------------------------------")
    print("Max Value: {0}\nMin Value: {1}\nZero Ref: {2}".format(
        epos.maxValue, epos.minValue, epos.zeroRef))
    print("---------------------------------------------")
    print("Moving into Zero Ref position....")
    epos.moveToPosition(epos.zeroRef)
    print('Done!')
    print("---------------------------------------------")

    # ---------------------------------------------------------------------------
    # Menu definitions
    # ---------------------------------------------------------------------------
    mainMenuSaveQC = Button("Save qc to file", 1)
    mainMenuReadQC = Button("Follow qc from file", 2)
    mainMenuMove = Button("Move to position", 3)
    mainMenuQuit = Button("Quit", 0)
    mainMenuButtons = [mainMenuSaveQC,
                       mainMenuReadQC, mainMenuMove, mainMenuQuit]
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
                    eposThread = threading.Thread(name="Save QC",
                                                  target=epos.saveToFile,
                                                  kwargs={'exitFlag': exitFlag})
                    eposThread.start()
                    print("Recording to file.")
                    input("Press Enter when done...")
                    exitFlag.set()
                    eposThread.join()
                elif val is 2:
                    pass
                elif val is 3:
                    pass
                else:
                    pass

    except KeyboardInterrupt as e:
        print('Got execption {0}... exiting now'.format(e))
    finally:
        exitFlag.set()  # in case any thread is still working
        epos.disconnect()
    return


if __name__ == '__main__':
    main()
