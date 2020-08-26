# -*- coding: utf-8 -*-
"""
Created on Thu Jun 25 14:25:43 2020

Real Time Stabilization Simulator

Using Euler Angles.

Units in SI, rad

@author: JRC

"""

#%%###########################

# Imports

import time
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


#%%###########################

# Variable Parameters

# Positional PID constants

P = 0.5
I = 0.1
D = 0.5
VFF = 0
AFF = 0

# Velocity PID Constants

P_vel = 0.1
I_vel = 0
D_vel = 0.1
# Angular PID Constants

P_ang = 10
I_ang = 0
D_ang = 1
VFF_ang = 0
AFF_ang = 0

angle_conv_const = 0.1

testtime = 60
buffersize = 1E3
# Drone Properties

#%%###########################

# System Constants


# Width of servo signal channel

signal_width = 1000


# Starting dt constant


initial_dt = 1.66E-3

grav_const = 9.805

acc_grav = np.array([[0],
                     [0],
                     [1]]) * grav_const
dt = initial_dt
#%%###########################

# System property imports

"""
In future, will be imported from a local file, but for now are just created.
"""

waypoints = pd.read_csv("waypoints.csv")

max_motor_thrust = 40
max_motor_speed = 30
motor_mass = 0.1
motor_tau = 0.2
#%%###########################

# Object Definitions


class Environment():
    """
    """

    def __init__(self, config, body_num, time_step):
        """
        """
        pr = pd.read_csv(config)
        self.grav_const = pr["Gravity"][body_num]
        global dt
        dt = time_step


class Motor():
    """
    General purpose motor object. Used to provide motor properties for an
    individual motor. The more motors a vehicle has, the more of these
    objects should be created.

    ORIENTATION IS UPRIGHT (-Z AXIS IN BODY FOR)
    """

    def __init__(self, thrust_max, omega_max, motor_mass,
                 motor_type='CW', thrust_curve='linear'):
        """

        Defines motor properties

        Parameters
        ----------
        thrust_max : Maximum motor thrust force
        omega_max : Maximum angular velocity
        motor_mass : Mass of the rotating portion of the motor
        motor_type : Defines motor direction. Default is
                    clockwise. Set to CCW for counter-clockwise
        thrust_curve : Defines the thrust curve of the motor output. Defaults
                        to linear configuration. Currently, no other types are
                        supported.


        Returns
        -------
        None.

        """

        self.tau = motor_tau
        self.thrust_max = thrust_max
        self.omega_max = omega_max
        self.motor_mass = motor_mass
        self.thrust_curve = thrust_curve
        self.motor_type = motor_type

        self.omega = 0
        self.thrust = 0

        self.position = np.array([[0],
                                  [0],
                                  [0]])

        self.motor_torque = np.array([[0],
                                      [0],
                                      [0]])

        if thrust_curve == 'linear':
            self.motor_const = self.thrust_max / self.omega_max

        self.force_b = np.array([[0],
                                 [0],
                                 [0]])

    def InToOut(self, input_param):
        """
        A function to update motor properties based on input signal.

        Parameters
        ----------
        input_param : Input signal sent to flight controller through signal
                        wire. Use actual signal value.

        Returns
        -------
        None.

        """

        self.ForceFind(input_param)
        self.PropertyCheck()
        self.ForceUpdate()

    def ForceFind(self, signal_in):
        """
        Finds force given input speed. Currently only uses a linear
        relationship, but should in future match to a thrust curve.


        Parameters
        ----------
        input_param : Takes input signal.

        Returns
        -------
        None

        """

        self.omega = (signal_in / signal_width) * self.omega_max
        self.thrust = self.motor_const*self.omega

    def PropertyCheck(self):
        """
        Reality check for motor properties.
        Ensures that output values cannot be higher than physically possible.
        Follows the motor properties update function.

        Returns
        -------
        None.

        """
        if self.thrust > self.thrust_max:
            self.thrust = self.thrust_max
        elif self.thrust < 0:
            self.thrust = 0
        else:
            pass

        if self.omega > self.omega_max:
            self.omega = self.omega_max
        elif self.omega < 0:
            self.omega = 0
        else:
            pass

    def ForceUpdate(self):
        """
        Auxilary function to update force in body reference.

        Returns
        -------
        None.

        """
        self.force_b = np.array([[0],
                                 [0],
                                 [-1]]) * self.thrust

    def PositionFix(self, x_b, y_b, z_b):
        """
        Function called to easily fix motor position in reference to
        flight computer.

        Parameters
        ----------
        x_b : X position of motor in body frame of reference
        y_b : Y position of motor in body frame of reference
        z_b : Z position of motor in body frame of reference

        Returns
        -------
        None.

        """
        self.position = np.array([[x_b],
                                  [y_b],
                                  [z_b]])


class MassTest():
    """
    An object to test large numbers of UAV simulations at the same time.
    """

    def __init__(self, import_file):
        """
        Creates an object containing a list of multiple UAV's
        in order to perform mass tests

        Parameters
        ----------
        import_file : Readable .csv file. Each row is an individual UAV
        and it's properties


        Returns
        -------
        None.

        """
        self.swarm_file = pd.read_csv(import_file)

        for i in range(len(self.swarm_file)):
            # Create objects here when you can create drone from a config file
            pass


class UAV():
    """

    All purpose UAV parent class.

    Creates positional and inertial data for the UAV, and simple functions for
    updating kinematic properties. Does NOT include force calculations from
    motors.

    NOT FOR USE FOR FIXED-WING AIRCRAFT!!!

    """

    def __init__(self, radius, mass):
        """


        Parameters
        ----------
        radius : Distance from center of mass to each motor. Assumed constant.
        x_v : Initialized velocity along the x axis. Assume Earth FOR
        y_v : Initialized velcity along the y axis. Assume Earth FOR
        z_v : Initialized veloctiy along the z axis. Assume Earth FOR
                Specificially, for z, the positive direction is towards the
                ground.

        Returns
        -------
        None.
        """
        self.repository_file_name = 'last_test.csv'

        # Basic Properties
        self.radius = radius
        self.mass = mass

        # Linear kinematic properties in body axis

        self.acc_b = np.array([[0],
                               [0],
                               [0]])

        self.vel_b = np.array([[0],
                               [0],
                               [0]])

        self.pos_b = np.array([[0],
                               [0],
                               [0]])

        # Linear kinematic properties in Earth Axis
        self.acc_e = np.array([[0],
                               [0],
                               [0]])

        self.vel_e = np.array([[0],
                               [0],
                               [0]])

        self.pos_e = np.array([[0],
                               [0],
                               [0]])

        # Forces and the principle of momentum

        self.force_grav = self.mass*acc_grav

        self.force_b = np.array([[0],
                                 [0],
                                 [0]])

        self.force_e = np.array([[0],
                                 [0],
                                 [0]])

        self.momentum_e = np.array([[0],
                                    [0],
                                    [0]])

        self.force_aero = np.array([[0],
                                    [0],
                                    [0]])

        # Rotational Kinematic properties

        # Placeholder values, fix these
        self.I_1 = 0.1
        self.I_2 = 0.2
        self.I_3 = 0.1

        self.angle = np.array([[0],
                               [0],
                               [0]])

        self.omega = np.array([[0],
                               [0],
                               [0]])

        self.omega_dot = np.array([[0],
                                   [0],
                                   [0]])

        column_names = ["Time",
                        "X Position"
                        "Y Position",
                        "Z Position",
                        "X Velocity",
                        "Y Velocity",
                        "Z Velocity",
                        "X Acceleration",
                        "Y Acceleration",
                        "Z Acceleration"
                        "Pitch",
                        "Roll",
                        "Yaw"]

        self.df = pd.DataFrame(columns=column_names)

        # Euler Angles
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        self.thrust = 0

        self.MotorInit(max_motor_thrust, max_motor_speed, motor_mass)
        self.time = 0

        self.prev_pos_err = 0
        self.int_pos = 0

        self.prev_pitch_err = 0
        self.int_pitch = 0

        self.prev_roll_err = 0
        self.int_roll = 0

        # PID integral constants

        self.int_pos_x = 0
        self.int_pos_xy = 0

        self.int_pos_y = 0
        self.int_pos_yx = 0

        # Initial waypoint settings
        self.setpoint_x = 0
        self.setpoint_y = 0
        self.setpoint_alt = 0

        self.waypoint_ticker = 0

        # Flags

        self.at_waypoint = False
        self.at_final = False

        self.last_time = 0
        self.checktime = 2
        self.min_dis = 0.2

    def UpdateAngle(self, roll, pitch, yaw):
        """
        Function to update UAV Euler angles.

        Uses Earth FOR

        Parameters
        ----------
        roll : Roll angle. Values between INSERT and INSERT
        pitch : Pitch angle. Values between INSERT and INSERT
        yaw : Yaw angle. Values between INSERT and INSERT

        Returns
        -------
        None.

        """
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw

    def MotorInit(self):
        """
        This is a dummy function.
        A function to generate motors in a specific layout.


        Returns
        -------
        None.

        """
        pass

    def Update(self):
        """
        Function that takes motor outputs, calculates all forces and torques,
        and updates the physics. Also records data for graphing purposes.

        Returns
        -------
        None.

        """
        self.SignalCheck()
        self.MotorForces()
        self.ForceAddition()
        self.NewtonsKinematics()

        self.MotorTorques()
        self.TorqueAddition()
        self.NewtonsRotational()

        self.RecordData()

    def MotorForces(self):
        """
        This is a dummy function.

        This function causes the system to create forces generated by motors.

        Returns
        -------
        None.

        """
        pass

    def MotorTorques(self):
        """
        This is a dummy function.

        This function causes the system to create torques caused by motors.

        Returns
        -------
        None.

        """
        pass

    def NewtonsKinematics(self):
        """
        Updates translational kinematic properties using kinematic equations.
        Uses the Earth FOR.

        Returns
        -------
        None.

        """

        # Note: += doesn't work for arrays for some weird reason
        self.acc_e = self.force_e / self.mass
        self.pos_e = self.vel_e*dt + self.pos_e + 0.5*self.acc_e*dt**2
        self.vel_e = self.acc_e * dt + self.vel_e
        # print(self.pos_e)

    def NewtonsRotational(self):
        """
        Calls functions to update rotational kinematic properties, and
        then to check the validity of the angles.

        Returns
        -------
        None.

        """
        BodyToEarth(self.total_torque, self)
        self.AngleCalc()
        self.AngleCheck()

    def AngleCalc(self):
        """
        Calculates rotational motion using Euler's equations.
        Uses deconstruction of Euler's equations along principal axes.

        Returns
        -------
        None.

        """

        # Please note. This is a messy way to do it, but otherwise makes an
        # array of arrays. Not ideal, but functional. Should fix later.

        # Creates local valeus to hold each angle, which are 1x1 arrays
        x = ((self.total_torque[0]
              - (self.I_3
                 - self.I_2)
              * self.omega[1] * self.omega[2]) / self.I_1)

        y = ((self.total_torque[1]
              - (self.I_1
                  - self.I_3)
              * self.omega[0] * self.omega[2]) / self.I_2)

        z = ((self.total_torque[2]
              - (self.I_2
                  - self.I_1)
              * self.omega[1] * self.omega[2]) / self.I_3)

        # Converts 1x1 arrays into scalars, and places them into the
        # angular velocity vector.

        self.omega_dot = np.array(
            [[x.item()],
             [y.item()],
             [z.item()]])

        self.angle = (self.angle
                      + self.omega_dot * 0.5 * dt**2
                      + self.omega * dt)

        # print(self.angle)

        self.omega = (self.omega
                      + self.omega_dot * dt)

        self.roll = self.angle[0].item()
        self.pitch = self.angle[1].item()
        self.yaw = self.angle[2].item()

    def AngleCheck(self):
        """
        Sanity check for Euler angles.
        Adjusts angles to remain in bounds

        Returns
        -------
        None.

        """
        check = False
        if self.yaw > np.pi:
            self.yaw -= 2*np.pi
            check = True
        elif self.yaw < -1*np.pi:
            self.yaw += 2*np.pi
            check = True
        else:
            pass

        if self.pitch > np.pi:
            self.pitch -= 2*np.pi
            check = True
        elif self.pitch < -1*np.pi:
            self.pitch += 2*np.pi
            check = True
        else:
            pass

        if self.roll > np.pi:
            self.roll -= 2*np.pi
            check = True
        elif self.roll < -1*np.pi:
            self.roll += 2*np.pi
            check = True
        else:
            pass

        if check is True:
            self.angle = np.array([[self.roll],
                                   [self.pitch],
                                   [self.yaw]])
        else:
            pass

    def ForceAddition(self):
        """
        This is a dummy function.

        Adds all motor forces together, as well as gravity and aerodynamic
        forces. Effectively a SIGMA F function.

        Returns
        -------
        None.

        """
        pass

    def IncreaseThrust(self):
        """
        Dummy function to increase thrust.

        Returns
        -------
        None.

        """
        pass

    def DecreaseThrust(self):
        """
        Dummy function to decrease thrust.

        Returns
        -------
        None.

        """
        pass

    def RecordData(self):
        """
        Basic function to record data to dataframe.
        Must be overwritten to include motor forces

        Returns
        -------
        None.

        """
        pass

    def Stabilize(self):
        """
        Calls individual correction functions to compensate for Euler angle
        instabilities.

        Returns
        -------
        None.

        """
        self.PitchControl(0)
        self.RollControl(0)
        self.YawCorrect()
        # self.Update()

    def Hover(self):
        pass

    def PitchCorrect(self):
        pass

    def RollCorrect(self):
        pass

    def YawCorrect(self):
        pass

    def SignalCheck(self):
        """
        Sanity check for motor signals.

        Returns
        -------
        None.

        """
        for i in range(len(self.signal)):
            if self.signal[i] > signal_width:
                self.signal[i] = signal_width
            if self.signal[i] < 0:
                self.signal[i] = 0

    def CheckWaypoint(self):
        xterm = (self.current_waypoint[0] - self.pos_e[0])**2
        yterm = (self.current_waypoint[1] - self.pos_e[1])**2
        zterm = (self.current_waypoint[2] - self.pos_e[2])**2
        dis = np.sqrt(xterm + yterm + zterm)

        if dis < self.min_dis:
            if self.at_final is True:
                print("Simulation Complete. Program will now end.")
                self.at_waypoint = True

            else:
                self.at_waypoint = True
                text = f'Arrived at {self.current_waypoint[0]}, \
                    {self.current_waypoint[1]}, {self.current_waypoint[2]}.'
                print(text)
                msg = f'Completed waypoint {self.waypoint_ticker} of \
                    {len(self.waypoints) -1}'
                print(msg)
                self.GetNewWaypoint()
        else:
            pass

    def GetNewWaypoint(self):
        """
        Retrieves next waypoint from waypoint in list

        Returns
        -------
        None.

        """

        self.waypoint_ticker += 1
        self.current_waypoint = np.array(
            [[self.waypoints["X"][self.waypoint_ticker]],
             [self.waypoints["Y"][self.waypoint_ticker]],
             [self.waypoints["Z"][self.waypoint_ticker]]]
            )
        self.Setpoint(self.current_waypoint[0],
                      self.current_waypoint[1],
                      self.current_waypoint[2])

        self.at_waypoint = False
        if self.waypoint_ticker == len(self.waypoints) - 1:
            self.at_final = True
        else:
            msg = f'Progressing to new waypoint at \
                {self.current_waypoint[0]}, {self.current_waypoint[1]}, \
                    {self.current_waypoint[2]}.'
            print(msg)

    def Setpoint(self, x, y, alt):
        self.setpoint_x, self.setpoint_y, self.setpoint_alt = x, y, alt

    def RunSimTimeStep(self):
        self.Update()
        self.Hover()
        self.Stabilize()
        self.time += dt
        if len(df) >= buffersize:
            ExportData(df, self.repository_file_name)
            update_message = f'Program is at simulation time {drone.time} at \
                waypoint {self.waypoint_ticker} of waypoint \
                    {len(self.waypoints)}'
            print(update_message)
        else:
            pass

        if self.time - self.last_time > self.checktime:
            self.CheckWaypoint()
            self.last_time = self.time
            print("Checking waypoints...")
        else:
            pass

    def RunSimulation(self):
        tic = time.time()
        self.ImportWaypoints()
        self.GetNewWaypoint()
        self.MainLoop()
        toc = time.time()
        tictoc = toc-tic
        msg = f'The simulation was completed after {tictoc} seconds.'
        print(msg)

    def MainLoop(self):

        while self.at_final is False and self.at_waypoint is False:
            self.RunSimTimeStep()

    def ImportWaypoints(self):
        self.waypoints = pd.read_csv('waypoints.csv')


class QuadX(UAV):
    """
    Perfectly square quadcopter, with motors in X layout.

    For reference, motor layout is as follows:

    0       1
        X
        X
    2       3

    Motor Directions:
    CW      CCW
        X
        X
    CCW     CW
    """

    def MotorInit(self, motor_thrust, motor_speed, motor_mass):
        """
        Defines motor positions and directions for this quadcopter format


        Parameters
        ----------
        motor_thrust : Maxiumum motor thrust
        motor_speed : Maxiumum motor speed
        motor_mass : Motor mass

        Returns
        -------
        None.

        """

        # Create motor objects

        self.motor_0 = Motor(motor_thrust, motor_speed, motor_mass, 'CW')
        self.motor_1 = Motor(motor_thrust, motor_speed, motor_mass, 'CCW')
        self.motor_2 = Motor(motor_thrust, motor_speed, motor_mass, 'CW')
        self.motor_3 = Motor(motor_thrust, motor_speed, motor_mass, 'CCW')

        # Define motor positions relative to flight computer
        x = self.radius/(2**0.5)
        y = self.radius/(2**0.5)

        self.motor_0.PositionFix(x, -1*y, 0)
        self.motor_1.PositionFix(x, y, 0)
        self.motor_2.PositionFix(-1*x, -1*y, 0)
        self.motor_3.PositionFix(-1*x, y, 0)

        # Initialize input signals
        self.signal = np.array([[0],
                                [0],
                                [0],
                                [0]])

        column_names = ["Time",
                        "X Position",
                        "Y Position",
                        "Z Position",
                        "X Velocity",
                        "Y Velocity",
                        "Z Velocity",
                        "X Acceleration",
                        "Y Acceleration",
                        "Z Acceleration",
                        "Pitch",
                        "Roll",
                        "Yaw",
                        "Motor 0 Signal",
                        "Motor 1 Signal",
                        "Motor 2 Signal",
                        "Motor 3 Signal",
                        "Motor 0 Thrust",
                        "Motor 1 Thrust",
                        "Motor 2 Thrust",
                        "Motor 3 Thrust"]

        global df
        df = pd.DataFrame(columns=column_names)
        df.to_csv(self.repository_file_name)

    def MotorForces(self):
        """
        Function which generates motor forces from signal inputs, then
        sums each force together.

        Returns
        -------
        None.

        """
        self.motor_0.InToOut(self.signal[0])
        self.motor_1.InToOut(self.signal[1])
        self.motor_2.InToOut(self.signal[2])
        self.motor_3.InToOut(self.signal[3])

        self.force_b = (self.motor_0.force_b
                        + self.motor_1.force_b
                        + self.motor_2.force_b
                        + self.motor_3.force_b)

        # Include created moments here too

    def MotorTorques(self):
        """
        Generates motor torques for each motor.

        Returns
        -------
        None.

        """
        self.motor_0.motor_torque = np.cross(self.motor_0.position,
                                             self.motor_0.force_b, axis=0)
        self.motor_1.motor_torque = np.cross(self.motor_1.position,
                                             self.motor_1.force_b, axis=0)
        self.motor_2.motor_torque = np.cross(self.motor_2.position,
                                             self.motor_2.force_b, axis=0)
        self.motor_3.motor_torque = np.cross(self.motor_3.position,
                                             self.motor_3.force_b, axis=0)

    def ForceAddition(self):
        """
        Adds forces from motors, gravity, and aerodynamic forces.

        Returns
        -------
        None.

        """
        self.force_e = BodyToEarth(self.force_b, self)
        self.force_e += self.force_grav + self.force_aero

        # Include sum of moments here too

    def TorqueAddition(self):
        """
        Sums torques from each motor together.

        Returns
        -------
        None.

        """
        self.total_torque = (self.motor_0.motor_torque
                             + self.motor_1.motor_torque
                             + self.motor_2.motor_torque
                             + self.motor_3.motor_torque)

    def RecordData(self):
        """
        Records all positional data at specific time step
        and adds as a new row to a dataframe.


        Returns
        -------
        None.

        """

        new_row = {"Time": self.time,
                   "X Position": float(self.pos_e[0]),
                   "Y Position": float(self.pos_e[1]),
                   "Z Position": float(self.pos_e[2]),
                   "X Velocity": float(self.vel_e[0]),
                   "Y Velocity": float(self.vel_e[1]),
                   "Z Velocity": float(self.vel_e[2]),
                   "X Acceleration": self.acc_e[0],
                   "Y Acceleration": self.acc_e[1],
                   "Z Acceleration": self.acc_e[2],
                   "Pitch": self.pitch,
                   "Roll": self.roll,
                   "Yaw": self.yaw,
                   "Motor 0 Signal": self.signal[0].item(),
                   "Motor 1 Signal": self.signal[1].item(),
                   "Motor 2 Signal": self.signal[2].item(),
                   "Motor 3 Signal": self.signal[3].item(),
                   "Motor 0 Thrust": self.motor_0.thrust,
                   "Motor 1 Thrust": self.motor_1.thrust,
                   "Motor 2 Thrust": self.motor_2.thrust,
                   "Motor 3 Thrust": self.motor_3.thrust}
        # Why is this global?
        # It saved some time by not adding to an object database
        # Means that only 1 simulation can be run at a time.
        # Could be reverted but not currently necessary

        global df
        df = df.append(new_row, ignore_index=True)

    def Hover(self):
        """
        Hover function.

        Returns
        -------
        None.

        """
        setpoint_vel = 0
        setpoint_acc = 0
        err = self.pos_e[2] - self.setpoint_alt
        self.int_pos += err*dt
        der = self.vel_e[2] - setpoint_vel
        self.signal = np.array([[1],
                                [1],
                                [1],
                                [1]]) * int(P * signal_width * err
                                            + I * signal_width * self.int_pos
                                            + D * signal_width * der
                                            + VFF * setpoint_vel
                                            + AFF * setpoint_acc)
        self.SignalCheck()
        # print(err)

    def Stabilize(self):
        """
        Calls individual correction functions to compensate for Euler angle
        instabilities.

        Returns
        -------
        None.

        """

        self.PitchCommand()
        self.RollCommand()

        self.YawCorrect()
        self.AngleCheck()
        self.SignalCheck()

    def PitchCommand(self):
        """
        Determines pitch setpoint using PID control for position, then calls
        the PitchControl function to execute that angle setpoint

        Returns
        -------
        None.

        """
        setpoint_vel = 0
        setpoint_acc = 0
        err_x = self.pos_e[0] - self.setpoint_x
        err_y = self.pos_e[1] - self.setpoint_y
        self.int_pos_x += err_x*dt
        self.int_pos_xy += err_y*dt
        der_x = self.vel_e[0] - setpoint_vel
        der_y = self.vel_e[1] - setpoint_vel
        setpoint_angle = ((P_vel * err_x
                          + I_vel * self.int_pos_x
                          + D_vel * der_x)
                          * np.cos(self.yaw)**2
                          + (P_vel * err_y
                             + I_vel * self.int_pos_xy
                             + D_vel * der_y)
                          * np.sin(self.yaw)**2)

        setpoint_angle = AngleBounds(setpoint_angle)
        # print(setpoint_angle)
        self.PitchControl(setpoint_angle)

    def PitchControl(self, setpoint_angle):
        # Look into PID position loop
        setpoint_vel = 0
        setpoint_acc = 0
        err = self.angle[1] - setpoint_angle
        self.int_pitch += err*dt
        der = self.omega[1] - setpoint_vel
        self.signal += np.array([[-1],
                                [-1],
                                [1],
                                [1]]) * int((P_ang * signal_width * err
                                             + I_ang * signal_width * self.int_pos 
                                             + D_ang * signal_width * der
                                             + VFF_ang * setpoint_vel
                                             + AFF_ang * setpoint_acc))
        # self.prev_pos_err = err
        # print(err)
        # self.Update()

    def RollCommand(self):

        setpoint_vel = 0
        setpoint_acc = 0
        err_x = self.pos_e[0] - self.setpoint_x
        err_y = self.pos_e[1] - self.setpoint_y
        self.int_pos_y += err_y * dt
        self.int_pos_yx += err_x * dt
        der_y = self.vel_e[1] - setpoint_vel
        der_x = self.vel_e[0] - setpoint_vel
        setpoint_angle = ((P_vel * err_y
                          + I_vel * self.int_pos_y
                          + D_vel * der_y)
                          * np.cos(self.yaw)**2
                          + (P_vel * err_x
                             + I_vel * self.int_pos_yx
                             + D_vel * der_x)
                          * np.sin(self.yaw)**2)
        setpoint_angle = AngleBounds(setpoint_angle)
        setpoint_angle = -1 * setpoint_angle
        # print(setpoint_angle)
        self.RollControl(setpoint_angle)

    def RollControl(self, setpoint_angle):

        setpoint_vel = 0
        setpoint_acc = 0
        err = self.angle[0] - setpoint_angle
        self.int_roll += err*dt
        der = self.omega[0] - setpoint_vel
        self.signal += np.array([[-1],
                                [1],
                                [-1],
                                [1]]) * int((P_ang * signal_width * err
                                             + I_ang * signal_width * self.int_pos
                                             + D_ang * signal_width * der
                                             + VFF_ang * setpoint_vel
                                             + AFF_ang * setpoint_acc))
        # self.prev_pos_err = err
        # print(err)
        # print(self.roll)
#%%###########################

# Function Definition


def YawConv(yaw):
    """


    Parameters
    ----------
    yaw : Current yaw angle. Earth FOR

    Returns
    -------
    yaw_conv_mat : Conversion matrix for yaw angle.

    """

    yaw_conv_mat = np.array([
                            [np.cos(yaw),     np.sin(yaw),    0],
                            [-1*np.sin(yaw),  np.cos(yaw),    0],
                            [0,               0,              1]]
                            )
    return yaw_conv_mat


def PitchConv(pitch):
    """


    Parameters
    ----------
    pitch : Current pitch angle. Earth FOR

    Returns
    -------
    pitch_conv_mat : Conversion matrix for pitch

    """
    pitch_conv_mat = np.array([[np.cos(pitch),     0,  -1*np.sin(pitch)],
                               [0,                 1,  0],
                               [np.sin(pitch),     0,  np.cos(pitch)]])
    return pitch_conv_mat


def RollConv(roll):
    """


    Parameters
    ----------
    roll : Current roll angle. Earth FOR

    Returns
    -------
    roll_conv_mat : Conversion matrix for roll

    """

    roll_conv_mat = np.array([[1,  0,                  0],
                              [0,  np.cos(roll),       np.sin(roll)],
                              [0,  -1*np.sin(roll),    np.cos(roll)]]
                             )
    return roll_conv_mat


def EarthToBody(original_vector, UAV):
    """
    Translates a vector from Earth FOR to Body FOR

    Parameters
    ----------
    original_vector : Self-explanatory.
    UAV : The drone itself, or the body it's translating to.

    Returns
    -------
    Vector in Body Axis FOR

    """
    interstage1 = np.dot(YawConv(UAV.yaw), original_vector)
    interstage2 = np.dot(PitchConv(UAV.pitch), interstage1)
    return np.dot(RollConv(UAV.roll), interstage2)


def BodyToEarth(original_vector, UAV):
    """
    Translates a vector from Body FOR to Earth FOR

    Parameters
    ----------
    original_vector : Self-explanatory.
    UAV : The drone itself, or the body it's translating to.

    Returns
    -------
    Vector in Earth Axis FOR

    """
    interstage1 = np.dot(RollConv(-1*UAV.roll), original_vector)
    interstage2 = np.dot(PitchConv(-1*UAV.pitch), interstage1)
    return np.dot(YawConv(-1*UAV.yaw), interstage2)


def BodyToStability(original_vector, UAV):
    """
    Currently empty function

    Parameters
    ----------
    original_vector : Vector to be transformed to stability FOR
    UAV : Input drone object, to take attitude data from.

    Returns
    -------
    None.

    """
    pass


def StabilityToBody(original_vector, UAV):
    """
    Currently empty function. Transforms stability vector to a

    Parameters
    ----------
    original_vector : Vector to be transformed to body FOR
    UAV : Input drone object, to take attitude data from.

    Returns
    -------
    None.

    """


def AngleBounds(setpoint_angle):
    """
    Ensures that the input angle does flip the aircraft over in an attempt to
    move sideways.

    Parameters
    ----------
    setpoint_angle : Setpoint angle calculated in the attitude control loop.

    Returns
    -------
    setpoint_angle : Setpoint angle in between the bounds of +- 0.5*pi

    """
    angle_max = 0.5*mt.pi

    if setpoint_angle > angle_max:
        setpoint_angle = angle_max
    if setpoint_angle < -1*angle_max:
        setpoint_angle = -1*angle_max

    return setpoint_angle


def plothusly(ax, x, y, xtitle='', ytitle='',
              datalabel='', title=''):
    """
    A little function to make graphing less of a pain.
    Creates a plot with titles and axis labels.
    Adds a new line to a blank figure and labels it.

    Parameters
    ----------
    ax : The graph object
    x : X axis data
    y : Y axis data
    xtitle : Optional x axis data title. The default is ''.
    ytitle : Optional y axis data title. The default is ''.
    datalabel : Optional label for data. The default is ''.
    title : Graph Title. The default is ''.

    Returns
    -------
    out : Resultant graph.

    """

    ax.set_xlabel(xtitle)
    ax.set_ylabel(ytitle)
    ax.set_title(title)
    out = ax.plot(x, y, zorder=1, label=datalabel)
    return out


def plothus(ax, x, y, datalabel=''):
    """
    A little function to make graphing less of a pain

    Adds a new line to a blank figure and labels it
    """
    out = ax.plot(x, y, zorder=1, label=datalabel)
    return out


def ExportData(df, outname="last_test.csv"):
    """
    Saves dataframe buffer to file. Saves an incredible amount of time.

    Parameters
    ----------
    df : Current vehicle dataframe
    outname : String of the desired filename. The default is "last_test.csv".

    Returns
    -------
    None.

    """
    df0 = pd.read_csv(outname)
    # This part removes the 'unnamed' column


    df0.drop(df0.columns[df0.columns.str.contains('unnamed', case=False)], axis=1, inplace = True)
    df0 = df0.append(df, ignore_index=True)
    df0.to_csv(outname)
    df.drop(df.index, inplace=True)


def CreateDataFrame():
    column_names = ["Time", 
                    "X Position",
                    "Y Position",
                    "Z Position",
                    "X Velocity",
                    "Y Velocity",
                    "Z Velocity",
                    "X Acceleration",
                    "Y Acceleration",
                    "Z Acceleration",
                    "Pitch",
                    "Roll",
                    "Yaw",
                    "Motor 0 Signal",
                    "Motor 1 Signal",
                    "Motor 2 Signal",
                    "Motor 3 Signal",
                    "Motor 0 Thrust",
                    "Motor 1 Thrust",
                    "Motor 2 Thrust",
                    "Motor 3 Thrust"]
    df = pd.DataFrame(columns = column_names)

drone = QuadX(0.5, 0.5)
# drone.RunSimulation()
#%%###########################

# Plotting results
# print(P, I, D)

# fig, zplot = plt.subplots()
# plothusly(zplot, drone.df["Time"], -1*drone.df["Z Position"], "Time in seconds",\
#           "Z position in metres", "Z Position", "Drone Position")
# plothus(zplot, drone.df["Time"], drone.df["Y Position"], "Y Position")
# plothus(zplot, drone.df["Time"], drone.df["X Position"], "X Position")
# plt.grid()
# plt.legend(loc="best")


# fig = plt.figure()
# threedplot = fig.add_subplot(111, projection='3d')
# threedplot.plot(drone.df["X Position"], drone.df["Y Position"], -1*drone.df["Z Position"])
# # threedplot.set_xlim(-3, 3)
# # threedplot.set_ylim(-3, 3)
# # threedplot.set_zlim(-10, 0)
# threedplot.set_xlabel('X Position')
# threedplot.set_ylabel('Y Position')
# threedplot.set_zlabel('Z Position')


# # fig, zvelplot = plt.subplots()
# # plothusly(zvelplot, drone.df["Time"], drone.df["Z Velocity"], "Time in seconds",\
# #           "Z velocity in metres/s", "Drone 1", "Z Velocity")

# # fig, zaccplot = plt.subplots()

# # plothusly(zaccplot, drone.df["Time"], drone.df["Z Acceleration"], "Time in seconds",\
# #           "Z velocity in metres/s", "Drone 1", "Z Acceleration")

# fig, signalplot = plt.subplots()
# plothusly(signalplot, drone.df["Time"], drone.df["Motor 0 Signal"], "Time",\
#           "Motor Signal", "Motor 0", "Motor Signals")
# plothus(signalplot, drone.df["Time"], drone.df["Motor 1 Signal"], "Motor 1")
# plothus(signalplot, drone.df["Time"], drone.df["Motor 2 Signal"], "Motor 2")
# plothus(signalplot, drone.df["Time"], drone.df["Motor 3 Signal"], "Motor 3")
# plt.grid()
# plt.legend(loc="best")

# fig, angleplot = plt.subplots()
# plothusly(angleplot, drone.df["Time"], drone.df["Pitch"], "Time in seconds", \
#           "Angle from neutral position in radians", "Pitch", "Euler angle plot")
# plothus(angleplot, drone.df["Time"], drone.df["Yaw"], "Yaw")
# plothus(angleplot, drone.df["Time"], drone.df["Roll"], "Roll")
# plt.grid()
# plt.legend(loc="best")