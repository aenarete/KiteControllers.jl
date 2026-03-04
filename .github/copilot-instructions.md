# Project Instructions for AI

## Overview
This is **KiteControllers.jl**, a Julia package providing discrete controllers for kite power systems. It is part of the [Julia Kite Power Tools](https://github.com/aenarete/KiteSimulators.jl) ecosystem.

## Language & Runtime
- All source code is **Julia** (v1.11 or v1.12)
- Package manager is **Pkg.jl** with a workspace layout (`Project.toml`, `examples/`, `test/`)
- The workspace root is the repo root; sub-projects live in `examples/` and `test/`

## Package Ecosystem
Key dependencies and their roles:
- **KiteUtils.jl** — shared types, settings, coordinate frames (re-exported)
- **WinchModels.jl** / **WinchControllers.jl** — winch dynamics and controllers (re-exported)
- **KiteModels.jl** — kite simulation models (used in examples and tests)
- **KitePodModels.jl** — kite pod models
- **DiscretePIDs.jl** — discrete PID controllers
- **Parameters.jl** — `@with_kw` structs for settings
- **YAML.jl** — configuration files in `data/`
- **StaticArrays.jl** — performance-critical array types

## Project Structure
```
src/                  # Main package source
  KiteControllers.jl  # Module entry point
  flightpathcontroller.jl
  flightpathcalculator2.jl
  flightpathplanner2.jl
  fpc_settings.jl / fpp_settings.jl
  kite_model.jl / kiteobserver.jl
  systemstatecontrol.jl
examples/             # Runnable examples (own Project.toml)
test/                 # Test suite (own Project.toml)
data/                 # YAML configuration files
output/               # Arrow log files from simulations
bin/                  # Helper scripts (create_sys_image, run_julia, etc.)
```

## Coding Conventions
- Use `@with_kw` structs from `Parameters.jl` for settings types
- Controller types implement `on_timer`, `calc_output`, `reset` interfaces
- State machines use Julia `@enum` (e.g. `SystemState`)
- Prefer `StaticArrays` (`SVector`, `MMatrix`) for fixed-size math
- Log data is stored as Apache Arrow files in `output/`
- Configuration is loaded from YAML files in `data/`

## Dependency Management
- The workspace uses **version-specific manifests**: `Manifest-v1.11.toml` and `Manifest-v1.12.toml`
- Compat bounds in `Project.toml` use the form `"major.minor.patch"` — only bump them when the required version is actually registered in the General registry
- When resolving `ERROR: empty intersection between Package@X and project compatibility Y`, check whether version Y is registered: `julia -e 'using Pkg; Pkg.Registry.update()'`, then inspect registry versions and delete the manifest if it is stale/conflicting (`rm Manifest-v1.12.toml; Pkg.resolve()`)
- If the manifest is stale/conflicting, deleting `Manifest-v1.12.toml` and running `Pkg.resolve()` forces a clean resolution

## Testing
Run tests from the repo root:
```julia
using Pkg; Pkg.test()
```
Or from the project directory:
```julia
include("test/runtests.jl")
```
Tests include Aqua.jl quality checks.

## System Image
A precompiled system image can be built with:
```bash
bin/create_sys_image
```
Launch Julia with it via `bin/run_julia`.
