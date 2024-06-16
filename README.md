# KiteControllers
[![Build Status](https://github.com/aenarete/KiteControllers.jl/actions/workflows/CI.yml/badge.svg?branch=main)](https://github.com/aenarete/KiteControllers.jl/actions/workflows/CI.yml?query=branch%3Amain)
[![Coverage](https://codecov.io/gh/aenarete/KiteControllers.jl/branch/main/graph/badge.svg)](https://codecov.io/gh/aenarete/KiteControllers.jl)

Discrete controllers for kite power systems.

This package is part of Julia Kite Power Tools, which consist of the following packages:
<p align="center"><img src="./docs/kite_power_tools.png" width="500" /></p>

## Installation
```julia
using Pkg
pkg"add KiteControllers"
```

## This package provides
### Utility functions
```
saturate(value, min_, max_)
```
Calculate a saturated value, that stays within the given limits.
```
wrap2pi(angle)
```
Convert an angle, given in radians in an infinite range to the range from -pi to pi

### Generic control components
This package contains some generic control components that are documented [here](./docs/components.md).

### Types that are not generic
```julia
CalcVSetIn              # component that calculates the set speed using soft switching
SpeedController         # controller for normal operation
LowerForceController    # controller when force near lower limit
UpperForceController    # controller when force near upper limit
WinchController         # winch controller, combining the three controllers above
WCSettings              # settings of the winch controller
WinchModel              # simplified model for unit testing

FlightPathController    # represents the flight path controller
FPCSettings             # settings of the flight path controller
FlightPathCalculator    # calculate the planned flight path
FlightPathPlanner       # execute the planned flight path
FPPSettings             # settings of the flight path planner
KiteModel               # simplified model for unit testing

SystemStateControl      # high level state machine, receives commands from the GUI
                        # and calls FlightPathPlanner and WinchController
```

## Flight path controller
FlightPathController as specified in chapter six of the [PhD thesis](https://research.tudelft.nl/en/publications/a-methodology-for-the-design-of-kite-power-control-systems) of Uwe Fechner.
```julia
FlightPathController
FPCSettings
on_control_command(fpc, attractor=nothing, psi_dot_set=nothing, radius=nothing, intermediate = true)
on_est_sysstate(fpc, phi, beta, psi, chi, omega, v_a; u_d=nothing, u_d_prime=nothing)
on_timer(fpc)
calc_steering(fpc, parking)
```
The control commands are usually received from the FlightPathPlanner, the output of the model or the system state estimator must call `on_est_systate()` each time step.
<p align="center"><img src="./docs/flight_path_controller_I.png" width="500" /></p>
<p align="center"><img src="./docs/flight_path_controller_II.png" width="500" /></p>

## Scientific background
[Flight path control of kite power systems in a turbulent wind environment](https://ieeexplore.ieee.org/document/7525563)

## Winch controller
For a kite power system, the reel-out speed of the winch must be controlled such that the
maximal tether force is never exceeded, while the reel-out speed should be optimized for
maximal power over the full cycle at wind speeds below rated wind speed. To keep the
kite controllable, also a minimal tether force limit has to be kept. Depending on the mode of operation, one of the following three controllers is used:
### Speed Controller
<p align="center"><img src="./docs/speed_controller.png" width="500" /></p>

### Lower Force Controller
<p align="center"><img src="./docs/lower_force_controller.png" width="500" /></p>

### Upper Force Controller
<p align="center"><img src="./docs/upper_force_controller.png" width="500" /></p>

### WinchController - Usage -
The WinchController combines the three controllers, mentioned above.
It can be operated in two modes of operation:
- position control
- power production

In position control mode it requires a set speed as input. Upper and lower force limits
are respected.
In power production mode it does not require any input but the measured tether force.
Output is the set speed of the asynchronous motor.

For a usage example look at the script [test_winchcontroller.jl](./test/test_winchcontroller.jl) .

### Flight Path Calculator (FPCA)
The FPCA is a stateless component that calculates the coordinates of the attractor points and turn points that
determine the flight path. In addition, it calculates the desired turn radius of the turns.

### Flight Path Planner (FPP)
The Flight Path Planner executes the planned flight path using a state machine.

### SystemStateControl (SSC)
While the Flight Path Planner is in charge of calculating the flight path during automated power production, this
component handles other system states, like fully manual operation, automated winch control combined with manual 
steering, parking etc.

## Hints for developers/ contributors
If you want to modify this package, create a fork on github and checkout the fork using git.
Then create a system image of all the packages, used by Kitecontrollers itself and the examples by running
the script `bin/create_sysimage --update`. If you are on Windows make sure you have git-bash installed because this
is a bash script. You can then launch julia with the command `bin/run_julia` which is fast. Try to run the 
test scripts from the folder test to understand what they are doing.
Now you can modify the source code according to your needs, restart Julia, run the relevant test script again and see 
the effect.
If you are happy, commit your changes and create a pull request. If you have any questions, please ask them
on https://discourse.julialang.org/ .

## Scientific background
[A Methodology for the Design of Kite-Power Control Systems](https://research.tudelft.nl/en/publications/a-methodology-for-the-design-of-kite-power-control-systems) Chapter 6.3 Winch control (WC)

## Related
- [Research Fechner](https://research.tudelft.nl/en/publications/?search=wind+Fechner&pageSize=50&ordering=rating&descending=true) for the scientific background of this code
- The meta package [KiteSimulators](https://github.com/aenarete/KiteSimulators.jl) which contains all packages from Julia Kite Power Tools.
- the packages [KiteModels](https://github.com/ufechner7/KiteModels.jl) and [WinchModels](https://github.com/aenarete/WinchModels.jl) and [AtmosphericModels](https://github.com/aenarete/AtmosphericModels.jl)
- the packages [KiteViewers](https://github.com/aenarete/KiteViewers.jl) and [KiteUtils](https://github.com/ufechner7/KiteUtils.jl)
