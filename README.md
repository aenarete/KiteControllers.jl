# KiteControllers
[![Build Status](https://github.com/aenarete/KiteControllers.jl/actions/workflows/CI.yml/badge.svg?branch=main)](https://github.com/aenarete/KiteControllers.jl/actions/workflows/CI.yml?query=branch%3Amain)
[![Coverage](https://codecov.io/gh/aenarete/KiteControllers.jl/branch/main/graph/badge.svg)](https://codecov.io/gh/aenarete/KiteControllers.jl)

Discrete controllers for kite power systems.

## Utility functions
```
saturate(value, min_, max_)
```
Calculate a saturated value, that stays within the given limits.
```
wrap2pi(angle)
```
Convert an angle, given in radian in an infinite range to the range from -pi to pi

## Types
```julia
Integrator

int = Integrator()  
int = Integrator(2,3) # integration constant, inital output  
reset(int)  
update(int, 2)        # input value  
on_timer(int)
```
## Flight path controller
FlightPathController as specified in chapter six of the PhD thesis of Uwe Fechner.
```julia
FlightPathController
FPCSettings
on_control_command(fpc, attractor=nothing, psi_dot_set=nothing, radius=nothing, intermediate = true)
on_est_sysstate(fpc, phi, beta, psi, chi, omega, v_a; u_d=nothing, u_d_prime=nothing)
on_timer(fpc)
calc_steering(fpc, parking)
```
<p align="center"><img src="./doc/flight_path_controller_I.png" width="500" /></p>
<p align="center"><img src="./doc/flight_path_controller_II.png" width="500" /></p>

## Winch controller
```
WinchController
```
Depending on the mode of operation, one of the following three controllers is used:
### Speed Controller
<p align="center"><img src="./doc/speed_controller.png" width="500" /></p>

### Lower Force Controller
<p align="center"><img src="./doc/lower_force_controller.png" width="500" /></p>

### Upper Force Controller
<p align="center"><img src="./doc/upper_force_controller.png" width="500" /></p>

## Related
- [Research Fechner](https://research.tudelft.nl/en/publications/?search=wind+Fechner&pageSize=50&ordering=rating&descending=true) for the scientic background of this code
- The meta package [KiteSimulators](https://github.com/aenarete/KiteSimulators.jl) which contains all packages from Julia Kite Power Tools.
- the packages [KiteModels](https://github.com/ufechner7/KiteModels.jl) and [WinchModels](https://github.com/aenarete/WinchModels.jl) and [AtmosphericModels](https://github.com/aenarete/AtmosphericModels.jl)
- the packages [KiteViewers](https://github.com/aenarete/KiteViewers.jl) and [KiteUtils](https://github.com/ufechner7/KiteUtils.jl)
