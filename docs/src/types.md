# Types

This page documents all public types exported by **KiteControllers.jl**.

## Settings

Settings structs are constructed once and passed to the corresponding controller.
They can be populated automatically from YAML configuration files in the `data/` folder
by passing `true` to the constructor (e.g. `FPCSettings(true)`).

```@docs
FPCSettings
FPPSettings
```

> The `WCSettings` type is documented in
> [WinchControllers.jl](https://github.com/aenarete/WinchControllers.jl).

## System State

```@docs
SystemState
```

The `SystemState` enum has the following values:

| Value | Integer | Description |
|:------|--------:|:------------|
| `ssManualOperation` | 0 | Manual control — no automation |
| `ssParking` | 1 | Kite is parked at high elevation |
| `ssPower` | 2 | Power phase in progress |
| `ssKiteReelOut` | 3 | Kite is being reeled out |
| `ssWaitUntil` | 4 | Waiting until kite reaches high elevation |
| `ssDepower` | 5 | Kite is being depowered |
| `ssIntermediate` | 6 | Transition phase before reel-out |
| `ssLaunching` | 7 | Kite launch sequence |
| `ssEmergencyLanding` | 8 | Emergency landing |
| `ssLanding` | 9 | Kite landing sequence |
| `ssReelIn` | 10 | Tether is being reeled in |
| `ssTouchdown` | 11 | Touchdown state |
| `ssPowerProduction` | 12 | Automated power production (figure-of-eight) |
| `ssWinchControl` | 13 | Automated winch control with manual steering |

## Flight Path Planner States

The `FPPS` (@enum) describes the sub-states of the flight path planner:

| Value | Integer | Meaning |
|:------|--------:|:--------|
| `INITIAL` | 0 | Initial state |
| `UPPER_TURN` | 1 | Upper turn after ascending |
| `LOW_RIGHT` | 2 | Low intermediate turn (right) |
| `LOW_TURN` | 3 | Turn around intermediate point |
| `LOW_LEFT` | 4 | Low intermediate turn (left) |
| `TURN_LEFT` | 5 | Turn at the left extreme of figure-of-eight |
| `FLY_RIGHT` | 6 | Straight-flight section to the right |
| `TURN_RIGHT` | 7 | Turn at the right extreme of figure-of-eight |
| `FLY_LEFT` | 8 | Straight-flight section to the left |
| `UP_TURN` | 9 | Ascending turn before depower |
| `UP_TURN_LEFT` | 10 | Left part of ascending turn |
| `UP_FLY_UP` | 11 | Ascending straight section |
| `DEPOWER` | 12 | Depower phase |
| `POWER` | 13 | Power phase |
| `PARKING` | 14 | Parking phase |

## Planner and Controllers

```@docs
FlightPathPlanner
FlightPathCalculator
FlightPathController
SystemStateControl
```

### WinchController (re-exported)

The following types are re-exported from
[WinchControllers.jl](https://github.com/aenarete/WinchControllers.jl):

- `WinchController` — main winch controller
- `SpeedController` — inner speed loop
- `LowerForceController` — lower tether force limit
- `UpperForceController` — upper tether force limit
- `Integrator` — discrete integrator block
- `UnitDelay` — unit delay block
- `RateLimiter` — rate limiter block
- `Mixer_2CH` — two-channel mixer
- `Mixer_3CH` — three-channel mixer
- `CalcVSetIn` — set-point calculator

## Observer

```@docs
KiteObserver
```
