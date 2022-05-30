module KiteControllers

using KiteUtils, Parameters, StaticArrays, NLsolve, Printf
import Base.reset

export Integrator, FlightPathController, FPCSettings, WCSettings    # types
export saturate, wrap2pi                                            # utility function
export reset, update, on_timer                                      # functions on Integrator
export on_control_command, on_est_sysstate, on_timer, calc_steering # functions on FlightPathController 
export calc_vro                                                     # functions for winch control

include("utils.jl")
include("flightpathcontroller.jl")
include("winchcontroller.jl")

end