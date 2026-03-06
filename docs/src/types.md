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

| Value | Description |
|:------|:------------|
| `ssManualOperation` | Manual control — no automation |
| `ssParking` | Kite is parked at high elevation |
| `ssPowerProduction` | Automated power production (figure-of-eight) |
| `ssReelIn` | Tether is being reeled in |
| `ssWinchControl` | Automated winch control with manual steering |
| `ssPower` | Power phase in progress |
| `ssKiteReelOut` | Kite is being reeled out |
| `ssIntermediate` | Transition phase before reel-out |
| `ssWaitUntil` | Waiting until kite reaches high elevation |
| `ssDepower` | Kite is being depowered |
| `ssLaunching` | Kite launch sequence |
| `ssLanding` | Kite landing sequence |
| `ssEmergencyLanding` | Emergency landing |
| `ssTouchdown` | Touchdown state |

## Flight Path Planner States

The `FPPS` (@enum) describes the sub-states of the flight path planner:

| Value | Meaning |
|:------|:--------|
| `POWER` | Power phase |
| `LOW_LEFT` / `LOW_RIGHT` | Low intermediate turns |
| `LOW_TURN` | Turn around intermediate point |
| `FLY_LEFT` / `FLY_RIGHT` | Straight-flight sections of figure-of-eight |
| `TURN_LEFT` / `TURN_RIGHT` | Turns at the extremes of figure-of-eight |
| `UP_TURN` | Ascending turn before depower |
| `UP_TURN_LEFT` | Left part of ascending turn |
| `UP_FLY_UP` | Ascending straight section |
| `UPPER_TURN` | Upper turn after ascending |
| `DEPOWER` | Depower phase |
| `PARKING` | Parking phase |

## Controllers

```@docs
FlightPathController
FlightPathCalculator
FlightPathPlanner
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
