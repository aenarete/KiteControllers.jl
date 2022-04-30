module KiteControllers

using KiteUtils, Parameters, StaticArrays, Printf
import Base.reset

export Integrator, FlightPathController, FPCSettings # types
export saturate, wrap2pi                                       # utility function
export reset, update, on_timer                                 # functions on Integrator
export on_control_command, on_est_sysstate                     # functions on FlightPathController 

include("utils.jl")
include("flightpathcontroller.jl")

end