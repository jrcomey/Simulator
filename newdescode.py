#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Feb 24 15:55:31 2021

@author: jack
"""

import UAVsym as usy
import numpy as np
import time
import matplotlib.pyplot as plt
import matplotlib as mpl
import matlab.engine

eng = matlab.engine.start_matlab()
print("Matlab booted")
# Defining motor thrust curve

max_thrust = 10   # Newtons

omega = np.linspace(0, 400, 1000)  # Values between 0 and 400 rad/s
thrust = max_thrust - max_thrust*np.exp(-omega/100)

# Creating base motor object

motor = usy.Motor(500)  # Set motor object with 100ms max PWM signal width
motor.SetTau(0.05)  # Set motor time constant in seconds
motor.SetThrustCurve(omega, thrust)  # Set motor thrust curve

# Defining UAV inertial properties
mass = 1  # kg
Ixx = 0.1  # kg-m^2
Iyy = 0.1  # kg-m^2
Izz = 0.1  # kg-m^2


num_motors = 4  # Number of UAV motors
clock_speed = 2.1E9  # Clock speed in Hz

B = np.array([[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
              [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
              [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
              [0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0],
              [0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
              [0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0],
              [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
              [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
              [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
              [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0],
              [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0],
              [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1]])


mixer = np.array([[0, 0, 0, 0],  # Empty
                  [0, 0, 0, 0],  # Empty
                  [0, 0, 0, 0],  # Empty
                  [0, 0, 0, 0],  # X Forces
                  [0, 0, 0, 0],  # Y Forces
                  [-1, -1, -1, -1],  # Z Forces
                  [0, 0, 0, 0],  # Empty
                  [0, 0, 0, 0],  # Empty 
                  [0, 0, 0, 0],  # Empty 
                  [0.25, -0.25, 0.25, -0.25],  # X Moments (Roll)
                  [0.25, 0.25, -0.25, -0.25],  # Y Moments (Pitch)
                  [0.25, -0.25, -0.25, 0.25]], dtype=float)  # Z Moments (Yaw)

drone = usy.UAV(mass, Ixx, Iyy, Izz, num_motors,
                motor, mixer, clock_speed)

# Set initial state to showcase control

drone.state_vector[2] = 10
drone.state_vector[1] = 5
drone.state_vector[0] = -5
drone.state_vector[8] = -0.5


drone.Setdt(0.001)  # Set time step size

# PID controls for angle, PD control for altitude position

K_P = 1  # P constant, angular
K_I = 0  # I constant, angular
K_D = 0  # D constant, angular
K_P_pos = 1  # P constant, altitude
K_D_pos = 0  # D constant, altitude
K_P_pos_xy = 1  # XY translational P constant
K_D_pos_xy = 0  # XY translational D constant

drone.SetPIDPD(K_P, K_I, K_D, K_P_pos, K_D_pos, K_P_pos_xy, K_D_pos_xy)

B = np.dot(np.eye(12), drone.mixer)

rho = 1

LQR = eng.lqr(matlab.double(drone.A.tolist()),
              matlab.double(B.tolist()),
              matlab.double(np.eye(12).tolist()),
              matlab.double((rho*np.eye(4)).tolist())
              )
print("LQR found")

drone.AlterControlMat(np.asarray(LQR))

# Main loop:

drone.RunSim(10)

# Exports data to pandas dataframe
df = drone.ExportData()

#%%###########################

# For plotting results only! Not contained in sample code.

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

plt.style.use("default")
plt.style.use("seaborn-bright")


params={#FONT SIZES
    'axes.labelsize':30,#Axis Labels
    'axes.titlesize':30,#Title
    'font.size':28,#Textbox
    'xtick.labelsize':22,#Axis tick labels
    'ytick.labelsize':22,#Axis tick labels
    'legend.fontsize':24,#Legend font size
    'font.family':'sans-serif',
    'font.fantasy':'xkcd',
    'font.sans-serif':'Helvetica',
    'font.monospace':'Courier',
    #AXIS PROPERTIES
    'axes.titlepad':2*6.0,#title spacing from axis
    'axes.grid':True,#grid on plot
    'figure.figsize':(12,12),#square plots
    # 'savefig.bbox':'tight',#reduce whitespace in saved figures#LEGEND PROPERTIES
    'legend.framealpha':0.5,
    'legend.fancybox':True,
    'legend.frameon':True,
    'legend.numpoints':1,
    'legend.scatterpoints':1,
    'legend.borderpad':0.1,
    'legend.borderaxespad':0.1,
    'legend.handletextpad':0.2,
    'legend.handlelength':1.0,
    'legend.labelspacing':0,}
mpl.rcParams.update(params)

fig, zplot = plt.subplots()
plothusly(zplot, df["Time"], -1*df["Z Position"], "Time in seconds",\
          "Position in metres", "Z Position", "NED Drone Position")
plothus(zplot, df["Time"], df["Y Position"], "Y Position")
plothus(zplot, df["Time"], df["X Position"], "X Position")
plt.grid()
plt.legend(loc="best")


fig, angleplot = plt.subplots()
plothusly(angleplot, df["Time"], df["Pitch"], "Time in seconds", \
              "Angle from neutral position in radians", "Pitch", "NED Aircraft Attitude")
plothus(angleplot, df["Time"], df["Yaw"], "Yaw")
plothus(angleplot, df["Time"], df["Roll"], "Roll")
plt.grid()
plt.legend(loc="best")