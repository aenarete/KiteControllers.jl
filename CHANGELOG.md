# Changelog

### [unreleased]
- adapt time_lapse according menu selection
- use 60 Hz refresh rate for 3x and 6x time lapse
- adapt text_mod such that the text update rate stays constant

#### Changed
- make use of log_level settings, print a lot less for log_level=0

### KiteControllers v0.2.2 - 2024-03-29

#### Added
- the example `autopilot.jl` was vastly improved
- a menu with 6 pre-defined plots was added to the GUI
- a statistics dialog was added
- saving and loading of log files added
- a second menu was added that allows to change the tolerance of the solver
- the new, pure Julia solver DFBDF is now the default. It is in average 4 times faster, uses only half the memory and is much more stable
