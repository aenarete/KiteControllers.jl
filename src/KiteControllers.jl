module KiteControllers

using Reexport
@reexport using KiteUtils
using NLsolve, Observables, Parameters, Printf, StaticArrays, WinchModels
using Pkg, StatsBase, StructTypes, YAML
@reexport using WinchControllers
import WinchControllers: calc_v_set, get_state, on_timer, reset, update
import JLD2

export FPCSettings, FlightPathController, Integrator, WCSettings    # types
export KiteModel, KiteObserver, WinchController
export CalcVSetIn, Mixer_2CH, Mixer_3CH, RateLimiter, UnitDelay
export LowerForceController, SpeedController, UpperForceController, Winch
export FlightPathCalculator, SystemState, SystemStateControl
export FLY_LEFT, FPPSettings, FlightPathPlanner, LOW_LEFT, LOW_RIGHT, LOW_TURN, POWER,
    TURN_LEFT 
export DEPOWER, FLY_RIGHT, PARKING, TURN_RIGHT, UPPER_TURN, UP_FLY_UP, UP_TURN, UP_TURN_LEFT
export @limit, saturate, wrap2pi                                       # utility function
export calc_output, get_state, on_timer, reset, select_b, select_c    # methods of Integrator, UnitDelay etc.
export calc_steering, on_control_command, on_est_sysstate, on_timer   # methods of FlightPathController 
export set_inactive, set_tracking, set_v_act, set_v_set, set_v_set_in # methods of SpeedController
export get_acc, get_speed, set_force                                  # methods of Winch
export get_f_err, set_f_set, set_reset, set_v_act, set_v_sw           # methods of LowerForceController
export calc_vro, set_vset_pc                                          # functions for winch control
export calc_v_set, get_status                          # methods of WinchController
export on_autopilot, on_new_systate, on_parking, on_reelin, on_stop   # methods of SystemStateControl
export get_depower, on_winchcontrol                                   # methods of SystemStateControl
export is_active, on_new_data, start                                  # methods of FlightPathPlanner
export ssManualOperation, ssParking, ssPowerProduction, ssReelIn
export observe!, update

abstract type AbstractForceController end
const AFC = AbstractForceController
const EPSILON = 1e-6
const FTOL    = 1e-8 # tolerance of residual for nonlinear solver

"""
    SystemState

Top-level system state enum used by [`SystemStateControl`](@ref).

Values: `ssManualOperation`, `ssParking`, `ssPowerProduction`, `ssReelIn`,
`ssWinchControl`, `ssPower`, `ssKiteReelOut`, `ssIntermediate`, `ssWaitUntil`,
`ssDepower`, `ssLaunching`, `ssLanding`, `ssEmergencyLanding`, `ssTouchdown`.
"""
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

include("fpc_settings.jl")
include("flightpathcontroller.jl")
include("kite_model.jl")
include("fpp_settings.jl")
include("kiteobserver.jl")
include("flightpathcalculator2.jl")
include("flightpathplanner2.jl")
include("systemstatecontrol.jl")

function copy_files(relpath, files)
    if ! isdir(relpath) 
        mkdir(relpath)
    end
    src_path = joinpath(@__DIR__, "..", relpath)
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
    src_path = joinpath(@__DIR__, "..", PATH)
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

"""
    copy_bin()

Copy the script `run_julia` to the folder "bin"
(it will be created if it doesn't exist).
"""
function copy_bin()
    PATH = "bin"
    mkpath(PATH)
    src_path = joinpath(@__DIR__, "..", PATH)
    cp(joinpath(src_path, "run_julia"), joinpath(PATH, "run_julia"), force=true)
    chmod(joinpath(PATH, "run_julia"), 0o774)
end

function install_examples(add_packages=true)
    copy_examples()
    copy_settings()
    Base.invokelatest(copy_control_settings)
    copy_bin()
    if add_packages
        Pkg.add("KiteViewers")
        Pkg.add("KiteUtils")
        Pkg.add("KitePodModels")
        Pkg.add("WinchModels")
        Pkg.add("KiteModels")
        Pkg.add("NativeFileDialog")
        Pkg.add("ControlPlots")
        Pkg.add("LaTeXStrings")
        Pkg.add("StatsBase")
        Pkg.add("Timers")
    end
    mkpath("output")
end


precompile(SystemStateControl, (WCSettings,))
precompile(on_parking, (SystemStateControl,))
precompile(on_autopilot, (SystemStateControl,))

end