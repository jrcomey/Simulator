"""

Example Code
"""

import UAVsym as usy
import numpy as np
import time

# Defining motor thrust curve

max_thrust = 10   # Newtons

omega = np.linspace(0, 400, 1000)
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

K_P = 10  # P constant, angular
K_I = 0.01  # I constant, angular
K_D = 4  # D constant, angular
K_P_pos = 100  # P constant, altitude
K_D_pos = 100  # D constant, altitude
K_P_pos_xy = 1  # XY translational P constant
K_D_pos_xy = 2  # XY translational D constant

drone.SetPIDPD(K_P, K_I, K_D, K_P_pos, K_D_pos, K_P_pos_xy, K_D_pos_xy)

# Main loop

t = 0  # ticker time
finish_time = 10  # max time
tic = time.time()  # For program timing purposes (MATLAB tictoc function)

# Main loop:

while t < finish_time:
    drone.RunSimTimeStep()  # Calls control loop and physics simulation
    t += drone.dt  # Advances time step for ticker
    drone.RecordData()  # Records current state at timestamp
    print(drone.signal)
toc = time.time()  # For program timing purposes (MATLAB tictoc function)
tictoc = toc-tic
string = f'State space model took {tictoc} seconds.'
print(string)

# Exports data to pandas dataframe
df = drone.ExportData()

#%%###########################

# For plotting results only! Not contained in sample code.

import matplotlib.pyplot as plt

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

fig, zplot = plt.subplots()
plothusly(zplot, df["Time"], -1*df["Z Position"], "Time in seconds",\
          "Position in metres", "Z Position", "NED Drone Position")
plothus(zplot, df["Time"], df["Y Position"], "Y Position")
plothus(zplot, df["Time"], df["X Position"], "X Position")
plt.grid()
plt.legend(loc="best")


fig, angleplot = plt.subplots()
plothusly(angleplot, df["Time"], df["Pitch"], "Time in seconds", \
              "Angle from neutral position in radians", "Pitch", "Euler angle plot")
plothus(angleplot, df["Time"], df["Yaw"], "Yaw")
plothus(angleplot, df["Time"], df["Roll"], "Roll")
plt.grid()
plt.legend(loc="best")