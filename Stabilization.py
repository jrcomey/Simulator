# -*- coding: utf-8 -*-
"""
Created on Thu Jun 25 14:25:43 2020

Real Time Stabilization Simulator

Using Euler Angles

Units in SI, rad

@author: JRC

"""


#%%###########################

# Imports

import time
import numpy as np
import math as mt
import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
plt.style.use('fast')

#%%###########################

# Variable Parameters

# PID Constants

P =     1   # unitless
I =     1   # unitless
D =     1   # unitless

angle_conv_const = 0.1

# Drone Properties

#%%###########################

# System Constants

signal_width = 100 # Width of servo signal channel
initial_dt = 1E-3 # Starting dt constant

grav_const = 9.805 # m*s**-2

acc_grav = np.array([[0],
                     [0],
                     [1]]) * grav_const
dt = initial_dt
#%%###########################

# System property imports

"""
In future, will be imported from a local file, but for now are just created.
"""

max_motor_thrust = 40 # N
max_motor_speed = 30 # rad/s
motor_mass = 0.1
motor_tau = 0.2 # s
#%%###########################

# Object Definitions
class Motor():
    """
    
    General purpose motor object. Used to provide motor properties for an
    individual motor. The more motors a vehicle has, the more of these
    objects should be created.
    
    ORIENTATION IS UPRIGHT (-Z AXIS IN BODY FOR)
    
    """
    
    def __init__(self, thrust_max, omega_max, motor_mass,\
                 motor_type = 'CW', thrust_curve = 'linear'):
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
        self.I_2 = 0.5
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
        
        self.df = pd.DataFrame(columns = column_names)
        # Euler Angles
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        self.thrust = 0

        self.MotorInit(max_motor_thrust, max_motor_speed, motor_mass)
        self.time = 0
        
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
                        -  (self.I_3 - self.I_2)*self.omega[1]*self.omega[2])
                        / self.I_1)
        y = ((self.total_torque[1] 
                        -  (self.I_1 - self.I_3)*self.omega[0]*self.omega[2])
                        / self.I_2)
        z = ((self.total_torque[2] 
                        -  (self.I_2 - self.I_1)*self.omega[1]*self.omega[2])
                        / self.I_3)
        
        # Converts 1x1 arrays into scalars, and places them into the 
        # angular velocity vector.
        
        self.omega_dot = np.array([[x.item()],
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
        if self.yaw > mt.pi:
            self.yaw -= 2*mt.pi
            check = True
        elif self.yaw < -1*mt.pi:
            self.yaw += 2*mt.pi
            check = True
        else:
            pass
        
        if self.pitch > mt.pi:
            self.pitch -= 2*mt.pi
            check = True
        elif self.pitch < -1*mt.pi:
            self.pitch += 2*mt.pi
            check = True
        else:
            pass
        
        if self.roll > mt.pi:
            self.roll -= 2*mt.pi
            check = True
        elif self.roll < -1*mt.pi:
            self.roll += 2*mt.pi
            check = True
        else:
            pass
        
        if check == True:
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
    def DecreaseThrust(self):
        """
        Dummy function to decrease thrust.

        Returns
        -------
        None.

        """

    def RecordData(self):
        """
        Basic function to record data to dataframe.
        Must be overwritten to include motor forces

        Returns
        -------
        None.

        """
    
    def Stabilize(self):
        """
        Calls individual correction functions to compensate for Euler angle
        instabilities.

        Returns
        -------
        None.

        """
        self.PitchCorrect()
        self.RollCorrect()
        self.YawCorrect()
    
    
   
    def Hover(self):
        """
        Hover function.

        Returns
        -------
        None.

        """
        if self.vel_e[2] > 0:
            self.IncreaseThrust()
        if self.vel_e[2] < 0:
            self.DecreaseThrust()
        else:
            pass
        self.Update()
        
        
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
        self.motor_1= Motor(motor_thrust, motor_speed, motor_mass, 'CCW')
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
        
        self.df = pd.DataFrame(columns = column_names)
        
        
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
        self.motor_0.motor_torque = np.cross(self.motor_0.position,\
                                             self.motor_0.force_b,axis=0)
        self.motor_1.motor_torque = np.cross(self.motor_1.position,\
                                             self.motor_1.force_b,axis=0)
        self.motor_2.motor_torque = np.cross(self.motor_2.position,\
                                             self.motor_2.force_b,axis=0)
        self.motor_3.motor_torque = np.cross(self.motor_3.position,\
                                             self.motor_3.force_b,axis=0)
        
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
        
    def IncreaseThrust(self):
        self.signal += 1
    
    def DecreaseThrust(self):
        self.signal -= 1
    
    def RecordData(self):
        new_row = {"Time" : self.time, 
                   "X Position" : self.pos_e[0],
                   "Y Position" : self.pos_e[1],
                   "Z Position": self.pos_e[2],
                   "X Velocity" : self.vel_e[0],
                   "Y Velocity" : self.vel_e[1],
                   "Z Velocity" : self.vel_e[2],
                   "X Acceleration" : self.acc_e[0],
                   "Y Acceleration" : self.acc_e[1],
                   "Z Acceleration" : self.acc_e[2],
                   "Pitch" : self.pitch,
                   "Roll" : self.roll,
                   "Yaw" : self.yaw,
                   "Motor 0 Signal" : self.signal[0].item(),
                   "Motor 1 Signal" : self.signal[1].item(),
                   "Motor 2 Signal"  : self.signal[2].item(),
                   "Motor 3 Signal"  : self.signal[3].item(),
                   "Motor 0 Thrust" : self.motor_0.thrust,
                   "Motor 1 Thrust" : self.motor_1.thrust,
                   "Motor 2 Thrust" : self.motor_2.thrust,
                   "Motor 3 Thrust" : self.motor_3.thrust}
        self.df = self.df.append(new_row, ignore_index=True)


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
                            [mt.cos(yaw),     mt.sin(yaw),    0],
                            [-1*mt.sin(yaw),  mt.cos(yaw),    0],
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
    pitch_conv_mat = np.array([[mt.cos(pitch),     0,  -1*mt.sin(pitch)],
                               [0,                 1,  0],
                               [mt.sin(pitch),     0,  mt.cos(pitch)]]
                            )
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
                              [0,  mt.cos(roll),       mt.sin(roll)],
                              [0,  -1*mt.sin(roll),    mt.cos(roll)]]
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
    pass

def StabilityToBody(original_vector, UAV):
    pass

def plothusly(ax, x, y, xtitle = '', ytitle ='', \
              datalabel = '', title=''):
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
    """
    
    """
    
    ax.set_xlabel(xtitle)
    ax.set_ylabel(ytitle)
    ax.set_title(title)
    out = ax.plot(x, y,zorder=1, label=datalabel)
    return out

def plothus(ax, x, y, datalabel = ''):
    """
    A little function to make graphing less of a pain
    
    Adds a new line to a blank figure and labels it
    """
    out = ax.plot(x, y, zorder=1, label=datalabel)
    return out

#%%###########################

# Test Code

drone = QuadX(0.25, 5)
drone.signal[0] = 0
drone.signal[1] = 0
drone.signal[2] = 0
drone.signal[3] = 0

tic = time.time()
while drone.time < 3:
    drone.Stabilize()
    drone.time += dt
    # print(drone.omega_dot)

toc = time.time()

tictoc = toc-tic
print(tictoc)
#%%###########################

# Plotting results
fig, zplot = plt.subplots()
plothusly(zplot, drone.df["Time"], -1*drone.df["Z Position"], "Time in seconds",\
          "Z position in metres", "Z Position", "Drone Position")
plothus(zplot, drone.df["Time"], drone.df["Y Position"], "Y Position")
plothus(zplot, drone.df["Time"], drone.df["X Position"], "X Position")
plt.grid()
plt.legend(loc="best")


fig = plt.figure()
threedplot = fig.add_subplot(111, projection='3d')
threedplot.plot(drone.df["X Position"], drone.df["Y Position"], -1*drone.df["Z Position"])

# fig, zvelplot = plt.subplots()
# plothusly(zvelplot, drone.df["Time"], drone.df["Z Velocity"], "Time in seconds",\
#           "Z velocity in metres/s", "Drone 1", "Z Velocity")

# fig, zaccplot = plt.subplots()

# plothusly(zaccplot, drone.df["Time"], drone.df["Z Acceleration"], "Time in seconds",\
#           "Z velocity in metres/s", "Drone 1", "Z Acceleration")

fig, signalplot = plt.subplots()
plothusly(signalplot, drone.df["Time"], drone.df["Motor 0 Signal"], "Time", "Motor Signal", "Motor 0", "Motor Signals")
plothus(signalplot, drone.df["Time"], drone.df["Motor 1 Signal"], "Motor 1")
plothus(signalplot, drone.df["Time"], drone.df["Motor 2 Signal"], "Motor 2")
plothus(signalplot, drone.df["Time"], drone.df["Motor 3 Signal"], "Motor 3")
plt.grid()
plt.legend(loc="best")

fig, angleplot = plt.subplots()
plothusly(angleplot, drone.df["Time"], drone.df["Pitch"], "Time in seconds", \
          "Angle from neutral position in radians", "Pitch", "Euler angle plot")
plothus(angleplot, drone.df["Time"], drone.df["Yaw"], "Yaw")
plothus(angleplot, drone.df["Time"], drone.df["Roll"], "Roll")
plt.grid()
plt.legend(loc="best")