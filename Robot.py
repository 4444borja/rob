#!/usr/bin/python
# -*- coding: UTF-8 -*-
from __future__ import print_function  # use python 3 syntax but make it compatible with python 2
from __future__ import division  # ''

import random

SIMULATED = True
SIMULATION_MAX_MOTOR_ERROR = 0.0

PLOT_SHOW_END = False
PLOT_SHOW_LIVE = False
LOGS_SAVE = False


import time  # import the time library for the sleep function
import datetime
import sys
import math
import plot_robot
import csv

# tambien se podria utilizar el paquete de threading
from multiprocessing import Process, Value, Array, Lock, Event


class Robot:
    def __init__(self, init_position=[0.0, 0.0, 0.0]):
        """
        Initialize basic robot params. \

        Initialize Motors and Sensors according to the set up in your robot
        """

        ######## UNCOMMENT and FILL UP all you think is necessary (following the suggested scheme) ########

        # Robot construction parameters
        self.R = 0.0408  # Radio de las ruedas (m)
        self.L = 0.1270  # Longitud eje (m)
        self.GEAR_RATIO = -3.6 / 2

        # odometry shared memory values
        self.x = Value('d', 0.0)
        self.y = Value('d', 0.0)
        self.th = Value('d', 0.0)
        self.finished = Value('b', 1)  # boolean to show if odometry updates are finished

        # target velocities that the robot wants to achieve
        self.target_w = 0
        self.target_v = 0

        # used to know when to plot when live-plotting
        self.plotCount = 0

        # used by the readSpeed() method
        self.lastEncoders = [0.0, 0.0]
        self.lastSpeedReadTime = time.time()

        self.motor_right_dps_set = Value('d', 0.0)
        self.motor_left_dps_set = Value('d', 0.0)

        # if we want to block several instructions to be run together, we may want to use an explicit Lock
        self.lock_odometry = Lock()

        self.event_odometryUpdated = Event()

        # odometry update period --> UPDATE value!
        self.P = 0.005

        # plot update period (0.03 -> 30FPS)
        self.PLOT_TIME = 0.03

        self.odometry_logs = []
        self.circle_logs = []

    def setSpeed(self, v, w):
        """
        Sets the speed of the motors to match the given linear and angular speeds of the robot
        :param v: Target linear speed of the robot (meters / second)
        :param w: Target angular speed of the robot (radians / second)
        :return: The linear (m/s) and angular speed (rad/s) targeted by the robot
        """

        # limit speed to 3 rad/s and 3 m/s
        max = 3
        if v < w:
            if w > max:
                w = max
                v = max*v / w
        else:
            if v > max:
                v = max
                w = max*w / v

        # Save target motor angular and linear speeds
        self.target_w = w
        self.target_v = v

        # Compute the speed that should be set in each motor
        speed_dps_right = math.degrees(v / self.R + w * self.L / (2 * self.R))
        speed_dps_left = math.degrees(v / self.R - w * self.L / (2 * self.R))

        # Save motor speed values
        self.motor_right_dps_set.value = speed_dps_right / self.GEAR_RATIO
        self.motor_left_dps_set.value = speed_dps_left / self.GEAR_RATIO

        return v, w

    def readSpeed(self):
        """
        Reads the speed from the motors and returns the speed from the right and left wheel respectively
        :return: Angular speeds of the right and left wheel respectively (radians / second)
        """

        # Calculate time difference since last encoder read
        t_ini = time.time()
        d_t = t_ini - self.lastSpeedReadTime
        self.lastSpeedReadTime = t_ini

        # Read last encoder values
        [lastEncoder1, lastEncoder2] = self.lastEncoders

    
        # Get random error values
        max_error_d = SIMULATION_MAX_MOTOR_ERROR
        max_error_i = SIMULATION_MAX_MOTOR_ERROR
        error_d = random.uniform(1.0 - max_error_d, 1.0 + max_error_d)
        error_i = random.uniform(1.0 - max_error_i, 1.0 + max_error_i)

        # Simulate motor encoder values with the random error in speed
        [encoder1, encoder2] = [lastEncoder1 + self.motor_right_dps_set.value * error_d * d_t,
                                lastEncoder2 + self.motor_left_dps_set.value * error_i * d_t]

        # Calculate wheel speeds
        wd = math.radians((encoder1 - lastEncoder1) * self.GEAR_RATIO) / d_t
        wi = math.radians((encoder2 - lastEncoder2) * self.GEAR_RATIO) / d_t

        # Save new encoder values
        self.lastEncoders = [encoder1, encoder2]

        return wd, wi

    def readOdometry(self):
        """
        Returns current value of odometry estimation
        :return: x (meters), y (meters) and theta (radians) of the robots position
        """
        self.lock_odometry.acquire()
        x, y, th = self.x.value, self.y.value, self.th.value
        self.lock_odometry.release()

        return x, y, th

    def setOdometry(self, x, y, th):
        """
        Sets the current value of odometry estimation
        :param x: X value in meters
        :param y: Y value in meters
        :param th: Theta value in radians
        :return:
        """
        self.lock_odometry.acquire()
        self.x.value = x
        self.y.value = y
        self.th.value = th
        self.lock_odometry.release()

    def startOdometry(self):
        """ This starts a new process/thread that will be updating the odometry periodically """
        self.finished.value = False
        self.p = Process(target=self.updateOdometry, args=())  # additional_params?))

        if PLOT_SHOW_LIVE:
            self.plot_process = Process(target=self.plotLive, args=())
            self.plot_process.start()

        self.p.start()
        print("PID: ", self.p.pid)

    def waitOdometryUpdated(self):
        """ Sends process to sleep until odometry is updated """
        self.event_odometryUpdated.wait()
        self.event_odometryUpdated.clear()

    def waitAngle(self, target_angle, threshold=0.075, waitForDifferenceToLower=True):
        """
        Sends process to sleep until a target angle (in radians) is reached within a specified threshold
        :param target_angle: The theta angle to be reached in radians
        :param threshold: Permitted difference between reached and target angles (in radians)
        :return: The actual reached theta angle (in radians)
        """

        # Read theta value
        self.lock_odometry.acquire()
        theta = self.th.value
        self.lock_odometry.release()

        # Calculate difference between theta and target angle
        angleDiff = math.pi - abs(abs(theta - target_angle) - math.pi)
        oldAngleDiff = angleDiff

        thresholdReached = angleDiff < threshold

        # Lock process if not target reached
        while ((not thresholdReached or (waitForDifferenceToLower and angleDiff < oldAngleDiff))
               and self.finished.value == False):
            oldAngleDiff = angleDiff
            # Awake process on odometry update
            self.waitOdometryUpdated()

            # Recalculate difference between theta and target angle
            self.lock_odometry.acquire()
            theta = self.th.value
            self.lock_odometry.release()
            angleDiff = math.pi - abs(abs(theta - target_angle) - math.pi)

            if angleDiff < threshold:
                thresholdReached = True

        return theta

    def waitDistance(self, distanceTarget):
        """
        Sends process to sleep until a target distance is reached from the actual position of the robot
        :param distanceTarget: The distance to be reached in meters
        :return: The actual distance reached (might be slightly larger than the target one)
        """

        # Lock odometry variables
        self.lock_odometry.acquire()
        x0 = self.x.value
        y0 = self.y.value
        self.lock_odometry.release()

        # Distance
        dist = 0.0

        # Lock process if target distance not reached
        while not (dist >= distanceTarget and self.finished.value == False):
            self.waitOdometryUpdated()

            # Update distance
            self.lock_odometry.acquire()
            x1 = self.x.value
            y1 = self.y.value
            self.lock_odometry.release()

            # Update distance value
            dist += math.sqrt((x1 - x0) * (x1 - x0) + (y1 - y0) * (y1 - y0))
            x0 = x1
            y0 = y1

        return dist

    def plotLive(self):
        """
        Plots the robot position every self.PLOT_TIME seconds until self.finished.value == True
        """
        tEnd = time.time()
        while not self.finished.value:
            # current processor time in a floating point value, in seconds
            tIni = time.time()

            # Read odometry variables
            [x, y, th] = self.readOdometry()

            # Show plot live
            plot_robot.dibrobot([x, y, th], 'red', 'p')
            plot_robot.draw()

            tEnd = time.time()

            waitTime = self.PLOT_TIME - (tEnd - tIni)
            if waitTime > 0:
                time.sleep(waitTime)

    def updateOdometry(self):  # , additional_params?):
        """
        Reads the motor values to update the odometry estimation of the robot.
        Logs the estimated position values in self.odometry_logs.
        Plots the robot position live if PLOT_SHOW_LIVE == True
        """

        tEnd = time.time()
        while not self.finished.value:
            # current processor time in a floating point value, in seconds
            tIni = time.time()
            d_t = tIni - tEnd

            if d_t > 0:
                # compute updates

                try:
                    # Read wheel angular speed
                    [wd, wi] = self.readSpeed()

                    # Calculate robot angular and linear speeds
                    w = (wd - wi) * self.R / self.L
                    v = self.R * (wd + wi) / 2

                    # Read odometry variables
                    [x, y, th] = self.readOdometry()

                    # Do odometry calculations
                    if w == 0:
                        x = x + v * math.cos(th) * d_t
                        y = y + v * math.sin(th) * d_t
                    else:
                        x = x + (v / w) * (math.sin(th + w * d_t) - math.sin(th))
                        y = y - (v / w) * (math.cos(th + w * d_t) - math.cos(th))
                        th = th + w * d_t
                        th = math.atan2(math.sin(th), math.cos(th))  # Normalize theta

                    # Set odometry variables
                    self.setOdometry(x, y, th)
                    #print("x: ",x," y: ",y," th: ",th)

                    # Notify odometry updated
                    self.event_odometryUpdated.set()

                    # Log odometry values
                    self.odometry_logs.append([x, y, th, time.time()])


                except IOError as error:
                    # print(error)
                    sys.stdout.write(error)

                tEnd = time.time()
                waitTime = self.P - (tEnd - tIni)
                if waitTime > 0:
                    time.sleep(waitTime)
            else:
                time.sleep(self.P)

        sys.stdout.write("Stopping odometry ... X=  %.2f, \
                Y=  %.2f, th=  %.2f \n" % (self.x.value, self.y.value, self.th.value))

        # Print odometry log file
        if LOGS_SAVE:
            # Open csv file
            f = open("./log.csv", 'w')

            # Write logs to CSV file
            with f:
                print("Printing logs to CSV file...")
                writer = csv.writer(f)
                writer.writerow(['x', 'y', 'theta', 'time'])
                for row in self.odometry_logs:
                    writer.writerow(row)

            # Close file
            f.close()

        plot_robot.plotLog(self.odometry_logs)
        

    # Stop the odometry thread.
    def stopOdometry(self):
        self.finished.value = True
        self.event_odometryUpdated.set()
