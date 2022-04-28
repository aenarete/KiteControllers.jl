module KiteControllers

using KiteUtils, Parameters
import Base.reset

export Integrator              # types
export saturate, wrap2pi       # utility function
export reset, update, on_timer # functions of Integrator

include("utils.jl")

end