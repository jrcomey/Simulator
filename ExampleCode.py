"""

Meaningless test code

"""

import NewTest as usy
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

FT = 5

x = np.linspace(0, 400, 1000)
y = FT - FT*np.exp(-x/100)
motor = usy.Motor(100)
motor.SetTau(0.0001)
motor.SetThrustCurve(x, y)


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

drone = usy.UAV(1, 0.1, 0.1, 0.1, 4, motor, mixer, 0)
drone.Setdt(0.001)  # Set time step size
drone.state_vector[2] = 0
drone.state_vector[6] = -1

drone.SetPIDPD(20, 0.1, 10, 0, 0)
t=0
tic = time.time()
while t < 20:
    # ticky = time.time()
    # drone.MotorControl()
    # drone.signal[0], drone.signal[1] = 1000, 1000
    # drone.Update()
    drone.RunSimTimeStep()
    # print(drone.state_vector)
    t += drone.dt
    # drone.RecordNumpy()
    drone.RecordData()
    # print(drone.int_vec)
    # tocky = time.time()
    # tickytocky = tocky-ticky
    # print(tickytocky)
    # print(drone.signal)
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