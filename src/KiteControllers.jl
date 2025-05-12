module KiteControllers

using Reexport
@reexport using KiteUtils
using WinchModels, Parameters, Observables, StaticArrays, NLsolve, Printf
using YAML, StructTypes, StatsBase, Pkg
import Base.reset
import JLD2

export Integrator, FlightPathController, FPCSettings, WCSettings    # types
export WinchController, KiteModel, KiteObserver
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
export update, observe!, moving_average

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
include("kiteobserver.jl")
include("flightpathcalculator2.jl")
include("flightpathplanner2.jl")
include("systemstatecontrol.jl")

function copy_files(relpath, files)
    if ! isdir(relpath) 
        mkdir(relpath)
    end
    src_path = joinpath(dirname(pathof(@__MODULE__)), "..", relpath)
    for file in files
        cp(joinpath(src_path, file), joinpath(relpath, file), force=true)
        chmod(joinpath(relpath, file), 0o774)
    end
    files
end

"""
    copy_examples()

Copy all example scripts to the folder "examples"
(it will be created if it doesn't exist).
"""
function copy_examples()
    PATH = "examples"
    if ! isdir(PATH) 
        mkdir(PATH)
    end
    src_path = joinpath(dirname(pathof(@__MODULE__)), "..", PATH)
    copy_files("examples", readdir(src_path))
end

function copy_control_settings()
    files = ["settings.yaml", "system.yaml", "fpc_settings_hydra20.yaml", "fpc_settings.yaml", 
             "fpp_settings_hydra20_426.yaml", "fpp_settings_hydra20_920.yaml", "fpp_settings_hydra20.yaml",
             "fpp_settings.yaml","gui.yaml.default", "hydra10_951.yml", "hydra20_426.yml", "hydra20_600.yml",
             "hydra20_920.yml", "settings_hydra20_600.yaml", "settings_hydra20_920.yaml", "settings_hydra20.yaml",
             "system_8000.yaml", "wc_settings_8000_426.yaml", "wc_settings_8000.yaml", "wc_settings.yaml"]
    dst_path = abspath(joinpath(pwd(), "data"))
    copy_files("data", files)
    set_data_path(joinpath(pwd(), "data"))
    println("Copied $(length(files)) files to $(dst_path) !")
end

function install_examples(add_packages=true)
    copy_examples()
    copy_settings()
    copy_control_settings()
    if add_packages
        Pkg.add("KiteViewers")
        Pkg.add("KiteUtils")
        Pkg.add("KitePodModels")
        Pkg.add("WinchModels")
        Pkg.add("KiteModels")
        Pkg.add("ControlPlots")
        Pkg.add("LaTeXStrings")
        Pkg.add("StatsBase")
        Pkg.add("Timers")
    end
end

precompile(SystemStateControl, (WCSettings,))
precompile(on_parking, (SystemStateControl,))
precompile(on_autopilot, (SystemStateControl,))

end