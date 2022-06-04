module KiteControllers

using KiteUtils, WinchModels, Parameters, StaticArrays, NLsolve, Printf, Mixers
import Base.reset

export Integrator, FlightPathController, FPCSettings, WCSettings    # types
export UnitDelay, RateLimiter, Mixer_2CH, Mixer_3CH, CalcVSetIn
export Winch, SpeedController
export saturate, wrap2pi                                              # utility function
export reset, calc_output, on_timer, select_b, select_c, get_state    # methods of Integrator, UnitDelay etc.
export on_control_command, on_est_sysstate, on_timer, calc_steering   # methods of FlightPathController 
export set_tracking, set_v_set, set_inactive, set_v_act, set_v_set_in # methods of SpeedController
export get_v_set_out                                                  # methods of SpeedController
export set_force, get_acc, get_speed                                  # methods of Winch
export calc_vro, set_vset_pc                                          # functions for winch control

include("utils.jl")
include("components.jl")
include("flightpathcontroller.jl")
include("wc_components.jl")
include("winchcontroller.jl")

end