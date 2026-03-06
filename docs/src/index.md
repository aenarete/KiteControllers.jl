# KiteControllers.jl

[![Build Status](https://github.com/aenarete/KiteControllers.jl/actions/workflows/CI.yml/badge.svg?branch=main)](https://github.com/aenarete/KiteControllers.jl/actions/workflows/CI.yml?query=branch%3Amain)
[![Coverage](https://codecov.io/gh/aenarete/KiteControllers.jl/branch/main/graph/badge.svg)](https://codecov.io/gh/aenarete/KiteControllers.jl)
[![DOI](https://zenodo.org/badge/486324875.svg)](https://zenodo.org/doi/10.5281/zenodo.13255245)
[![Aqua QA](https://raw.githubusercontent.com/JuliaTesting/Aqua.jl/master/badge.svg)](https://github.com/JuliaTesting/Aqua.jl)

**KiteControllers.jl** provides discrete controllers for kite power systems. It is part of the
[Julia Kite Power Tools](https://github.com/aenarete/KiteSimulators.jl) ecosystem.

![Kite Power Tools](https://github.com/aenarete/WinchModels.jl/blob/main/docs/kite_power_tools.png?raw=true)

## Overview

The package implements the control hierarchy described in the PhD thesis of Uwe Fechner:

| Component | Description |
|:----------|:------------|
| [`SystemStateControl`](@ref) | Top-level state machine coordinating all sub-controllers |
| [`FlightPathPlanner`](@ref) | Plans the figure-of-eight flight path and manages phase transitions |
| [`FlightPathCalculator`](@ref) | Calculates attractor points and angular speed of the kite |
| [`FlightPathController`](@ref) | PID/NDI controller translating the planned path into a steering signal |
| `WinchController` | Controls tether reel-in/-out speed (re-exported from WinchControllers.jl) |

The winch components (`WinchController`, `Winch`, `SpeedController`,
`LowerForceController`, `UpperForceController`) are re-exported from
[WinchControllers.jl](https://github.com/aenarete/WinchControllers.jl).

## Installation

```julia
using Pkg
pkg"add KiteControllers"
```

To add examples and their dependencies:

```julia
using KiteControllers
KiteControllers.install_examples()
```

## Quick Start

```julia
using KiteControllers

# Load settings
wcs  = WCSettings(true)
fcs  = FPCSettings(true)
fpps = FPPSettings(true)

# Create top-level controller
ssc = SystemStateControl(wcs, fcs, fpps; u_d0=0.01, u_d=0.25, v_wind=10.0)

# Set the state machine to power production mode
on_autopilot(ssc)

# Each simulation time step:
# 1. Feed new kite state
on_new_systate(ssc, sys_state)

# 2. Query the winch speed set-point
v_set = calc_v_set(ssc)
```

## Package Structure

```
KiteControllers.jl
├── SystemStateControl   – top-level state machine
├── FlightPathPlanner    – path planning (chapter 5 of PhD thesis)
│   └── FlightPathCalculator  – attractor & omega calculation
│       └── FlightPathController  – PID/NDI steering (chapter 6)
└── WinchController      – reel-in/-out speed control (re-exported)
```

## Contents

```@contents
Pages = ["types.md", "functions.md", "examples.md"]
Depth = 2
```
