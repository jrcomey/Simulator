"""

Meaningless test code

"""

import UAVsym as usy
import numpy as np
import time
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

max_thrust = 5  # Newtons

omega = np.linspace(0, 400, 1000)
thrust = max_thrust - max_thrust*np.exp(-omega/100)

motor = usy.Motor(100)  # Set motor object with 100ms max PWM signal width
motor.SetTau(0.01)  # Set motor time constant in seconds
motor.SetThrustCurve(omega, thrust)  # Set motor thrust curve


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
                  [0.25, -0.25, -0.25, 0.25]])  # Z Moments (Yaw)

drone = usy.UAV(mass, Ixx, Iyy, Izz, num_motors,
                motor, mixer, clock_speed)
drone.Setdt(0.001)  # Set time step size
# drone.state_vector[2] = 0
# drone.state_vector[6] = -1

drone.SetPIDPD(10, 1, 1, 100, 10)
t=0
finish_time = 20
tic = time.time()
while t < finish_time:
    drone.RunSimTimeStep()
    t += drone.dt
    drone.RecordData()
toc = time.time()
tictoc = toc-tic
print(drone.state_vector)
string = f'State space model took {tictoc} seconds.'
print(string)

df = drone.ExportData()

#%%###########################

# Plotting results

fig, zplot = plt.subplots()
plothusly(zplot, df["Time"], -1*df["Z Position"], "Time in seconds",\
          "Z position in metres", "Z Position", "Drone Position")
plothus(zplot, df["Time"], df["Y Position"], "Y Position")
plothus(zplot, df["Time"], df["X Position"], "X Position")
plt.grid()
plt.legend(loc="best")


# fig = plt.figure()
# threedplot = fig.add_subplot(111, projection='3d')
# threedplot.plot(df["X Position"], df["Y Position"], -1*df["Z Position"])
# # threedplot.set_xlim(-3, 3)
# # threedplot.set_ylim(-3, 3)
# # threedplot.set_zlim(-10, 0)
# threedplot.set_xlabel('X Position')
# threedplot.set_ylabel('Y Position')
# threedplot.set_zlabel('Z Position')


# fig, zvelplot = plt.subplots()
# plothusly(zvelplot, df["Time"], df["Z Velocity"], "Time in seconds",\
#           "Z velocity in metres/s", "Drone 1", "Z Velocity")

# fig, zaccplot = plt.subplots()

# plothusly(zaccplot, df["Time"], df["Z Acceleration"], "Time in seconds",\
#           "Z velocity in metres/s", "Drone 1", "Z Acceleration")


fig, angleplot = plt.subplots()
plothusly(angleplot, df["Time"], df["Pitch"], "Time in seconds", \
          "Angle from neutral position in radians", "Pitch", "Euler angle plot")
plothus(angleplot, df["Time"], df["Yaw"], "Yaw")
plothus(angleplot, df["Time"], df["Roll"], "Roll")
plt.grid()
plt.legend(loc="best")