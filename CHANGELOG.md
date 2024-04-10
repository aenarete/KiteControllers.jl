# Changelog

### [unreleased]
#### Changed
- use 60 Hz refresh rate for 3x, 6x, 9x and 12x time lapse
- refactoring: add type KiteApp to the script autopilot.jl
- make use of log_level settings, print a lot less for log_level=0
- refactoring: split `flightpathplanner.jl` in `flightpathplanner.jl` and `flightpathcalculator.jl`
- created the simplified components `flightpathplanner2.jl` and `flightpathcalculator2.jl`;  
  they are easier to understand and contain a lot less hard-coded constants

#### Added
- adapt time_lapse according to menu selection
- adapt text_mod such that the text update rate stays constant
- make it possible to change `t_sim` using the GUI
- add the plots `plot_elev_az2` and `plot_side_view2` that omit the first power cycle
- add the plots `plot_elev_az3`, `plot_side_view3` and `plot_front_view3` that omit the first two power cycles
- add time series of `fig8` to `plot_control`
- add the file `kiteobservers.jl` which provides the type `KiteObserver` and methods to 
determine deviations from the desired flight path
- add the script `learning.jl` that provides the method `train()` that implements iterative learning
of a vector of flight path corrections 

### KiteControllers v0.2.2 - 2024-03-29
#### Changed
- the new, pure Julia solver DFBDF is now the default. It is in average 4 times faster, uses only half the memory and is much more stable. In my tests, for rel\_tol $=0.0005$ the solver induced error of the peak tether force and the harvested energy was always $<0.1$%.

#### Added
- the example `autopilot.jl` was vastly improved
- a menu with 6 pre-defined plots was added to the GUI
- a statistics dialog was added to the menu
- saving and loading of log files added to the menu
- a second menu was added that allows to change the tolerance of the solver
