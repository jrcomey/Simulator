# UAVsym

UAVsym is a python package designed to provide a virtual testing ground for multicopter Unmanned Arial Vehicles (UAV's) by modelling vehicle behaviour by using a state space model. It is intended for use as a design tool to verify UAV flight behaviour.
The package supports all multirotor formats with static motors (e.g. no tricopters). Motors can be in any orientation.

## Installation and setup

The package can be downloaded through pip, or through $OTHER_WAY

On Linux, the package is downloadable using:

```bash
sudo pip install $UAVsym
```

Note that the package has not yet been distributed, and cannot be downloaded yet.

## Usage

In this simulation, the UAV is built by first configuring a motor object. First create the motor object, setting the maximum PWM signal width. Then define a thrust curve through a series of points. The package will convert the data into a 6-th degree polynomial estimation for speed of calculation. The example uses a decaying thrust curve, but any data can be used.

```python
import UAVsym as usy
import numpy as np

max_thrust = 5  # Newtons

omega = np.linspace(0, 400, 1000)
thrust = max_thrust - max_thrust*np.exp(-omega/100)

motor = usy.Motor(100)  # Set motor object with 100ms max PWM signal width
motor.SetTau(0.001)  # Set motor time constant in seconds
motor.SetThrustCurve(omega, thrust)  # Set motor thrust curve
```


From there, a UAV object is created, assuming each of the motors on the aircraft are identical. The physics calculation is performed using a state space model. The aircraft itself is defined through a motor mixer matrix, which defines forces and moments on the local UAV body as a function of their magnitude. The example below shows a quadcopter with all motors pointing upwards, and each motors 25cm away on both the x and y axes.
The UAV itself can be configured using predertimined inertial properties.

```python
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

```

Once the vehicle is configured, you can begin the simulation. During the re-write of the package, only only time-steps are currently included. This will be addressed shortly.
```python
t = 0  # ticker time
finish_time = 10  # max time

while t < finish_time:
    drone.RunSimTimeStep()  # Calls control loop and physics simulation
    t += drone.dt  # Advances time step for ticker
    drone.RecordData()  # Records current state at timestamp

# Exports data to pandas dataframe
df = drone.ExportData()
```

The package also supports PID tuning for desired vehicle behaviour.

If you have any questions or bug reports, the github page for this project is https://github.com/jrcomey/Simulator. I can also be reached at jrcomey@ucdavis.edu.

## Demo

Included in the package is a demo of how this package should be used, under ExampleCode.py.


# Liscense

Link to liscense here
