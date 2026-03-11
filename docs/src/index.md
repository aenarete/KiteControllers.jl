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

```@raw html
<details>
<summary>Installation of Julia</summary>
```

If you do not have Julia installed yet, please read [Installation](https://github.com/aenarete/KiteSimulators.jl/blob/main/docs/Installation.md).

```@raw html
</details>

<details>
<summary>Installation as package</summary>
```

### Installation of KiteControllers as package

It is suggested to use a local Julia environment. You can create it with:
```bash
mkdir myproject
cd myproject
julia --project=.
```
(don't forget typing the dot at the end), and then, on the Julia prompt enter:
```julia
using Pkg
pkg"add KiteControllers#main"
```
You can run the tests with:
```julia
using Pkg
pkg"test KiteControllers"
```
To add the examples and install the packages needed by the examples, run:
```julia
using KiteControllers
KiteControllers.install_examples()
exit()
```

```@raw html
</details>

<details>
<summary>Installation using git</summary>
```

### Installation of KiteControllers using git

In most cases -- if you want to modify, tune and understand kite controllers -- it is better to check out this project from git. You can do this with:
```bash
git clone https://github.com/aenarete/KiteControllers.jl.git
cd KiteControllers.jl
```
Then, run the install script and optionally create a system image:
```bash
cd bin
./install
./create_sys_image
cd ..
```
The startup time without system image is about 30s, with system image 5s, but creating a system image takes 15 minutes or more.

On Linux you can install the `autopilot` GUI app as normal application with the command `bin/install_app`.

```@raw html
</details>
```

### Running the first example

You can now start Julia with `./bin/run_julia` and execute the autopilot or any other of the examples with:
```julia
include("examples/autopilot.jl")
```
Clicking on the "RUN" button starts the simulation. When the simulation is finished, you can click on the "OK"
button at the top left. After some time a plot of the flight should appear. You can use the drop-down menu
on the left to select other plots or the statistics. You can also load or save log files or projects.
A project consists of a kite/ physical system, controller settings and a wind condition.

You get a menu with all the examples by typing:
```julia
menu()
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
