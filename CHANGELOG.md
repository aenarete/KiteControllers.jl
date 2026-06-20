# Changelog

## KiteControllers v0.2.29 - 2026-06-21
### Changed
- **Switched from ControlPlots to MakieControlPlots** across all examples, tests, and scripts. Main advantages: thread-safe plotting, better integration with the Makie ecosystem.
- Removed `bin/install_controlplots` script (no longer needed — MakieControlPlots uses matplotlib via CondaPkg or system install)
- Updated `bin/install` to check for MakieControlPlots instead of ControlPlots; removed macOS QtAgg/matplotlib installation logic
- Updated CI workflow
- Disabled inline type hints in JETLS configuration
- Added type annotations (`::FPPS`, `::SystemState`) to switch functions in `flightpathplanner2.jl` and `systemstatecontrol.jl` to fix JETLS warnings
- Updated default manifests

## KiteControllers v0.2.28 - 2026-05-14
### Added
- `--update`, `--yes`, `--help` command-line options to `bin/install`
- `CondaPkg.toml` for managing matplotlib and pyqt via CondaPkg/pixi
- `JULIA_PYTHONCALL_EXE` environment variable in `setup_env` pointing to CondaPkg Python
- CondaPkg directories to `.gitignore`

### Changed
- switched from PyCall/PyPlot to PythonCall/CondaPkg for ControlPlots (bump ControlPlots to 0.3). Main advantage: The plotting library is now thread-safe.
- `bin/install_controlplots` now uses CondaPkg instead of Conda/PyCall; default backend changed to CondaPkg
- `bin/install`: added `--update` mode that removes manifests and runs `Pkg.update()`
- `bin/install`: improved error handling with automatic retry on resolve failures
- `bin/install`: passes `--yes` flag to `install_controlplots` in non-interactive mode
- `bin/setup_env`: improved OpenSSL handling (use LD_LIBRARY_PATH instead of force-preloading libcrypto)
- `bin/setup_env`: added libexpat handling for CondaPkg/pixi Python 3.13+
- `bin/setup_env`: suppress verbose CondaPkg logs by default
- `bin/run_julia`: use `-t auto` instead of `-t 1` to utilize all CPU threads
- `bin/create_sys_image`: removed Python/matplotlib detection (no longer needed with CondaPkg)
- updated default manifests

### Fixed
- fixed OpenSSL symbol mismatch by not force-preloading libcrypto in launcher wrappers

## KiteControllers v0.2.26 - 2026-05-04
### Changed
- improved `bin/install` script
- parking examples (`parking_4p.jl`, `parking_wind_dir.jl`) now work with 9% wind turbulence
- added elevation and force plots to `parking_wind_dir.jl`
- updated default manifests

### Fixed
- fixed OpenSSL issue in `bin/setup_env`
- fixed `bin/run_julia` for Julia 1.11

## KiteControllers v0.2.25 - 2026-05-03
### Added
- the functions `set_default_turbulence` and `get_default_turbulence`
- the menu entry `set_turbulence`
- the script `bin/install_kaimon_database`
- the documentation page `projects.md`
- the test `test-default_turbulence.jl`
- `run_julia` uses `Revise` now by default; Revise must be installed globally

### Changed
- the example `parking_wind_dir` and all other relevant examples now use the default turbulence
- the install script now installs Revise globally
- added `default_turbulence` to `gui.yaml`

### Fixed
- fixed a typo in `docs/src/projects.md`
- fixed a warning in `examples/parking_wind_dir.jl`

## KiteControllers v0.2.24 - 2026-05-02
### Added
- the function `menu_learning()` and the script `menu_learning.jl`
- the page `Learning Control` to the documentation
- the script `mwe_06.jl` for listing the exported symbols
- the script `select_project` to provide an interactive way to select the active project

### Changed
- the file `kiteobserver.jl`. It uses now a different method to determine the average elevation of a figure of eight.
- the script learning.jl was renamed to learn_corrections.jl
- the script now stores the corrections directly in the yaml file of the flight path planner
- it has better error handling and is much more robust

## KiteControllers v0.2.23 - 2026-04-27
### Fixed
- updated TagBot.yml

## KiteControllers v0.2.22 - 2026-04-27
### Fixed
- the script `autopilot` did not use the renamed system image

## KiteControllers v0.2.21 - 2026-04-26
### Changed
- improved `install` script to do a complete installation also with Julia 1.11
- improved `create_sys_image` script to use less memory on Julia 1.12 and to suppress one warning
- bump KiteModels to 0.11.8: This version provides full turbulence support
- bump KiteUtils to 0.11.7
- default is now 100% turbulence relative to Cabauw, NL for 6 m/s wind speed
. reduced default order of solver from 4 to 3 to improve stability
### Fixed
- decreased v_min to 0.10 m/s (the speed when the brake gets released) to fix sporadic winch controller failures
- initial state of hydra20_426.yml

## KiteControllers v0.2.20 - 2026-03-15
### Changed
- fixed new JETLS warnings, also in the tests
- improved script `test_flightpathcontroller1.jl`, added unit tests
- improved the `plot_timing`, it now shows a line at the real-time limit
- improve documentation
- bump ControlPlots to 0.2.14 to fix an MacOS bug
- bump KiteViewers to 0.5.2 to avoid GUI artifact when starting the autopilot
- add shutdown guard to avoid showing a 2D plot when closing the `autopilot.jl` window
- re-wrote the script update_default_manifest
### Added
- support for MacOS
- the script `test/test_menu.jl` for running manual controller tests

## KiteControllers v0.2.19 - 2026-03-12
### Changed
- TestEnv is not used any more
- use subprojects
- add `.JETLSConfig.toml.default` for new linter
- fix all JETLS warnings
- update packages; the performance increased a lot
- bump KiteUtils to 0.11.4
- the `run_julia` script accepts now a script name as parameter
- improve script `install`
- improved installation instructions
- documented more functions of `FlightPathPlanner`
- the examples `tune_1p.jl` and `tune_4p.jl` where updated. They now use the NOMAD optimizer.
- the performance improved by a factor of about 7. The GUI can now 24x real-time.
  In batch mode, 180x realtime can be achieved

### Added
- add `Documenter.jl` based documentation
- add script `batch_pilot.jl` for simulating a set of projects
- add Bash script `batch_pilot` as driver for `batch_pilot`. It supports the --help command. It can run 180x realtime.
- add Bash script `batch_plot` to plot the results and calculate statistics. It supports the --help command.
- add Bash script `jetls` to run the static code analysis
- add Bash script `jetls_examples` to run the static code analysis on the files in the `examples` folder.

## KiteControllers v0.2.18 2025-07-14
### Changed
- update KiteModels to 0.9.0; this version supports turbulent wind fields
### Fixes
- fix broken dependencies (KiteUtils)

## KiteControllers v0.2.17 2025-06-20
### Changed
- remove all code related to the winch controller and use the package WinchControllers instead
- bump KiteUtils and KiteModels
- update yaml files for new version of KiteUtils

## KiteControllers v0.2.16 2025-05-13
### Added
- the function `install_examples()`

## KiteControllers v0.2.15 2025-05-12
### Fixed
- `plot_main` had wrong labels
### Added
- add `yaw_rate` and `steering` to AoA plot
### Changed
- remove outdated example `tune_4p.jl` from menu
- the script create_sys_image now checks if the current Julia version is too old
- bump KiteUtils to 0.10.5
- bump KiteModels to 0.7.3

## KiteControllers v0.2.14 2025-04-22
### Fixed
- disable multithreading in `run_julia` to avoid crashes related to PyPlot
### Changed
- update the script `create_sys_image` to support both Julia 1.10 and Julia 1.11
- add Aqua.jl for quality insurance
- remove unused dependencies found by Aqua
- bump KiteUtils to 0.10.3
- bump KiteModels to 0.7

## KiteControllers v0.2.13 2025-01-22
### Fixed
- add new version of `FFTW` as dependency to fix Windows issue

## KiteControllers v0.2.12 2025-01-21
- change the point `zenith` to 79° elevation, 0° azimuth
- change the initialization in autopilot.jl to work better for high wind speeds

## KiteControllers v0.2.11 2025-01-16
### Changed
- use KiteModels v0.6.14, which defines the azimuth angle and the orientation differently and make the controllers and examples work with the new definitions
- bump `KiteUtils` to v0.9.6 The new version has new fields in the `SysState` struct that are used for logging.
- the constructor `SystemStateControl()`now needs the additional parameter `v_wind`
- the constructors `WCSettings()`, `FPCSettings()` and `FPPSettings()` now have the new argument `update`. If true,
  then the settings are loaded from the corresponding `yaml` file.
- do not use the function `update_sys_state!()` any longer because it is buggy
- reexport KiteUtils
- when executing `bin/run_julia`, always execute `using KiteControllers` before displaying the REPL
- make use of the environment variable "USE_V9"; if set, use a different (proprietary) settings file
- improve example `parking_wind_dir.jl`
### Fixed
- fixed logging of the height and X, Y and Z
- plotting of the height is fixed in `autopilot.jl`
### Added
- the menu with the examples can now started by typing `menu()`
- add the script `parking_wind_dir.jl` that tests the parking controller when the wind direction is changing
- add the script `parking_controller.jl` which implements a dual-loop parking controller. The inner loop controls the turn rate. It has an excellent performance.
- add the script `test/menu.jl` which allows to execute the manual tests, that display plots and fix the tests

## KiteControllers v0.2.10 - 2024-09-07
### Changed
- the script `create_sys_image` is now installing matplotlib if required
- removed calls to se() to be sure the correct settings from the variable set are used
- fixed the wrong polars in the yaml settings for the 20 m² kite
- use the new, correct methods for calculating the polars in autopilot.jl
- added the new fields needed for the new winch controller release
- bump KiteUtils to 0.7.9
- bump KiteModels to 0.6.6
### Fixed
- fix all failing test scripts
- fix most of the example, and the script `menu.jl`to run the examples interactively

## KiteControllers v0.2.9 - 2024-08-07
### Changed
- bump KiteUtils to 0.7.4
- bump KiteModels to 0.6.3
- fix some examples and the script `create_sys_image`
- explain different installation methods in README.md

## KiteControllers v0.2.8 - 2024-07-28
### Changed
- bump KiteUtils to 0.7.2
- bump WinchModels to 0.3.2
- bump KiteViewers to 0.4.16
- adapt WinchController to new WinchModel interface
- fix tests

## KiteControllers v0.2.7 - 2024-07-12
### Fixed
- fix control_plot on Windows (added sleep)
- fix cycle counting in FPP
### Added
- add function plot_timing2()
- add lower limit for C2 of 2.0
### Changed
- bump KiteViewers.jl to 0.4.14
- upgrade GLFW to latest version (the original issue is fixed now)

## KiteControllers v0.2.6 - 2024-07-02
### Changed
- downgrade GLFW_jll to fix an issue on Ubuntu 24.04 and on Mac
### Added
- add section kps4-3l to all settings files to be compatible with the latest KiteUtils package

## KiteControllers v0.2.5 - 2024-06-26
### Added
- copyright disclaimer from TU Delft
### Changed
- fix #35, save the default log file in the output folder

## KiteControllers v0.2.4 - 2024-06-18
### Changed
- add the field `corr_vec` to all `fpp_settings_xxx.yaml` files
- modify `kiteobserver.jl` to use this correction vector
- the projects `hydra10_951` and `hydra20_600` `hydra20_920` work correctly now; key change: set k_c2_high to 6.0
- new plot control_plot_II
- remove compatibility with Julia 1.9, version 1.10 is the only supported version now

## KiteControllers v0.2.3 - 2024-05-06
### Changed
- use 60 Hz refresh rate for 3x, 6x, 9x and 12x time lapse
- refactoring: add type KiteApp to the script autopilot.jl
- make use of log_level settings, print a lot less for log_level=0
- refactoring: split `flightpathplanner.jl` in `flightpathplanner.jl` and `flightpathcalculator.jl`
- created the simplified components `flightpathplanner2.jl` and `flightpathcalculator2.jl`;  
  they are easier to understand and contain a lot less hard-coded constants

### Added
- adapt time_lapse according to menu selection
- adapt text_mod such that the text update rate stays constant
- make it possible to change `t_sim` using the GUI
- add the plots `plot_elev_az2` and `plot_side_view2` that omit the first power cycle
- add the plots `plot_elev_az3`, `plot_side_view3` and `plot_front_view3` that omit the first two power cycles
- add the plot `aerodynamics` that plots the lift-over-drag of the kite
- add time series of `fig8` to `plot_control`
- add the file `kiteobservers.jl` which provides the type `KiteObserver` and methods to determine deviations from the desired flight path
- add the script `learning.jl` that provides the method `train()` that implements iterative learning of a vector of flight path corrections

## KiteControllers v0.2.2 - 2024-03-29
### Changed
- the new, pure Julia solver DFBDF is now the default. It is on average 4 times faster, uses only half the memory and is much more stable. In my tests, for rel\_tol $=0.0005$ the solver induced error of the peak tether force and the harvested energy was always $<0.1$%.

### Added
- the example `autopilot.jl` was vastly improved
- a menu with 6 pre-defined plots was added to the GUI
- a statistics dialog was added to the menu
- saving and loading of log files added to the menu
- a second menu was added that allows to change the tolerance of the solver
