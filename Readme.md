This is a currently a placeholder introduction.

# UAVsym

UAVsym is a python package designed to provide a virtual testing ground for multicopter Unmanned Arial Vehicles (UAV's) by modelling vehicle behaviour by using a state space model. 
It was created after I was unable to find an appropriate space to test aerodynamic properties of an airplane on my university campus. 
Currently, the package supports a single quadcopter format (X style, all motors equidistant from CoM), but will be updated to include various layouts, as well as other rotary wing formats such as helicopters, and eventually, fixed wing aircraft. 

## Installation and setup

The package can be downloaded through pip, or through $OTHER_WAY

On Linux, the package is downloadable using:

```bash
sudo pip install $THE_PACKAGE_NAME
```
## Usage

In this simulation, the UAV is built by first configuring a motor object. In an ideal situation, you should have an experimentally determined thrust curve and time response for your motor, but as this package is designed to reduce the need for physical testing, the motor object can be configured from base properties.

```python
motor_sample = UAVsym.Motor() # Creates a generic motor object
motor_sample.DefineThrustCurve(SympyExpression) # Mathematically defines thrust curve
motor_sample.SetTau(0.2) # Sets time constant in seconds
```
Alternatively, thrust curves can be defined from a pre-determined dataset.

```python
motor_sample.ImportThrustCurve(thrust_curve.csv) # Imports pre-defined thrust curve
```

From there, a UAV object is created, assuming each of the motors on the aircraft are identical. The physics calculation is performed using a state space model, with the matrix layout taken from Dr. $NAME's research on UAV's.
The UAV itself can be configured using predertimined inertial properties, but the program also allows for determination of inertial properties part-by-part.

```python
drone = UAVsym.UAV(motor_sample) # Creates a UAV using previous motor object

drone.ConfigureInertia(numpyThreeByThree) # Defines Inertial Matrix and verifies it
drone.SetPID(P, I, D) # Sets PID constants
```

Once the vehicle is configured, you can begin testing it for individual properties, or test for a wide range at once. These properties will be printed to terminal, and saved to a file.
```python
drone.TestOvershoot()
drone.TestAll()
```

The package also supports PID tuning for desired vehicle behaviour.

If you have any questions or bug reports, the github page for this project is https://github.com/jrcomey/Simulator. I can also be reached at jrcomey@ucdavis.edu.

## Demo

Attatched here is a demo of how this package should be used. Hopefully this should provide a good example. 

$DEMO_LINK_HERE


# Liscense

Link to licsense here
