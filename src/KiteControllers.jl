module KiteControllers

using KiteUtils, Parameters, StaticArrays, NLsolve, Printf
import Base.reset

export Integrator, FlightPathController, FPCSettings, WCSettings    # types
export UnitDelay, RateLimiter, Mixer_2CH
export saturate, wrap2pi                                            # utility function
export reset, calc_output, on_timer                                 # functions on Integrator, UnitDelay etc.
export on_control_command, on_est_sysstate, on_timer, calc_steering # functions on FlightPathController 
export calc_vro                                                     # functions for winch control

include("utils.jl")
include("components.jl")
include("flightpathcontroller.jl")
include("winchcontroller.jl")

end