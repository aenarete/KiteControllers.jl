# Functions

This page documents all public functions exported by **KiteControllers.jl**.

## SystemStateControl

These methods drive the top-level state machine.

```@docs
on_autopilot
on_parking
on_reelin
on_stop
on_winchcontrol
on_new_systate(::SystemStateControl, ::Any)
calc_v_set(::SystemStateControl)
get_depower
```

## FlightPathPlanner

The flight path planner manages flight-phase transitions and forwards kite state
to the flight path controller.

```@docs
start(::FlightPathPlanner, ::Any)
is_active(::FlightPathPlanner)
get_state(::FlightPathPlanner)
on_new_systate(::FlightPathPlanner, ::Any, ::Any, ::Any, ::Any, ::Any, ::Any)
on_new_data
```

## FlightPathController

The flight path controller translates planned trajectory information into a
steering signal `u_s`.

```@docs
on_control_command
on_est_sysstate
calc_steering
```

## WinchController (re-exported)

The following functions are re-exported from
[WinchControllers.jl](https://github.com/aenarete/WinchControllers.jl).
See that package's documentation for details.

```@docs
calc_v_set
```

## Observer

```@docs
observe!
```
