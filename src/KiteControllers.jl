module KiteControllers

using KiteUtils, WinchModels, Parameters, Observables, StaticArrays, NLsolve, Printf
using YAML, StructTypes
import Base.reset

export Integrator, FlightPathController, FPCSettings, WCSettings    # types
export WinchController, KiteModel
export UnitDelay, RateLimiter, Mixer_2CH, Mixer_3CH, CalcVSetIn
export Winch, SpeedController, LowerForceController, UpperForceController
export FlightPathCalculator, SystemStateControl, SystemState
export FlightPathPlanner, FPPSettings, POWER, LOW_LEFT, FLY_LEFT, TURN_LEFT, LOW_RIGHT, LOW_TURN 
export FLY_RIGHT, TURN_RIGHT, UP_TURN, UP_TURN_LEFT, UP_FLY_UP, UPPER_TURN, DEPOWER, PARKING
export saturate, @limit, wrap2pi                                       # utility function
export reset, calc_output, on_timer, select_b, select_c, get_state    # methods of Integrator, UnitDelay etc.
export on_control_command, on_est_sysstate, on_timer, calc_steering   # methods of FlightPathController 
export set_tracking, set_v_set, set_inactive, set_v_act, set_v_set_in # methods of SpeedController
export get_v_set_out, get_v_error                                     # methods of SpeedController
export set_force, get_acc, get_speed                                  # methods of Winch
export set_v_act, set_reset, set_f_set, set_v_sw, get_f_err           # methods of LowerForceController
export get_f_set_low                                                  # methods of LowerForceController
export get_f_set_upper                                                # methods of UpperForceController
export calc_vro, set_vset_pc                                          # functions for winch control
export calc_v_set, get_set_force, get_status                          # methods of WinchController
export on_autopilot, on_parking, on_reelin, on_stop, on_new_systate   # methods of SystemStateControl
export on_winchcontrol, get_depower                                   # methods of SystemStateControl
export ssParking, ssPowerProduction, ssReelIn, ssManualOperation
export update

abstract type AbstractForceController end
const AFC = AbstractForceController
const EPSILON = 1e-6
const FTOL    = 1e-8 # tolerance of residual for nonlinar solver

@enum SystemState begin ssManualOperation; ssParking; ssPower; ssKiteReelOut; 
                        ssWaitUntil;    # wait until high elevation
                        ssDepower;
                        ssIntermediate; # ssPower, before ssKiteReelOut
                        ssLaunching; ssEmergencyLanding; ssLanding; ssReelIn; ssTouchdown; ssPowerProduction;
                        ssWinchControl  # automated winch control, manual steering 
                  end

function __init__()
    if isdir(joinpath(pwd(), "data")) && isfile(joinpath(pwd(), "data", "system.yaml"))
        set_data_path(joinpath(pwd(), "data"))
    end
end

include("utils.jl")
include("components.jl")
include("fpc_settings.jl")
include("flightpathcontroller.jl")
include("kite_model.jl")
include("wc_settings.jl")
include("wc_components.jl")
include("winchcontroller.jl")
include("fpp_settings.jl")
include("flightpathcalculator.jl")
include("flightpathplanner.jl")
include("systemstatecontrol.jl")

precompile(SystemStateControl, (WCSettings,))
precompile(on_parking, (SystemStateControl,))
precompile(on_autopilot, (SystemStateControl,))

end