module KiteControllers

using KiteUtils, Parameters, StaticArrays
import Base.reset

export Integrator, FlightPathController              # types
export saturate, wrap2pi                             # utility function
export reset, update, on_timer                       # functions of Integrator

include("utils.jl")
include("flightpathcontroller.jl")

end