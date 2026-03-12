"""
Settings of the FlightPathController.

Loaded from the YAML file returned by `fpc_settings()` (typically `data/fpc_settings.yaml`).
Construct with `FPCSettings(true; dt)` to auto-load from disk, or `FPCSettings(; dt)` for defaults.

# Fields
- `dt::Float64`: controller sample period [s].
- `log_level::Int64`: logging verbosity (0 = silent, 2 = default).
- `prn::Bool`: enable general debug printing.
- `prn_ndi_gain::Bool`: print NDI gain each step.
- `prn_est_psi_dot::Bool`: print estimated turn rate each step.
- `prn_va::Bool`: print apparent wind speed each step.
- `use_radius::Bool`: use turn-radius control mode instead of heading-rate mode.
- `use_chi`: use course angle χ as the controlled variable instead of heading ψ.
- `reset_int1::Bool`: reset the main integrator to the last estimated turn rate on state entry.
- `reset_int2::Bool`: reset the D-term integrator at the second time step after a state transition.
- `reset_int1_to_zero::Bool`: reset the main integrator to zero (overrides `reset_int1`).
- `init_opt_to_zero::Bool`: initialize the root-finder with zero instead of the previous solution.
- `p`: proportional gain of the PID controller.
- `i`: integral gain of the PID controller.
- `d`: derivative gain of the PID controller.
- `gain`: common scaling factor applied to all of P, I and D.
- `c1`: NDI model coefficient c₁ (default ≈ 0.0613).
- `c2`: NDI model coefficient c₂ (default ≈ 1.226).
- `k_c1`: correction factor for the NDI c₁ term; increase to enlarge the turn radius.
- `k_c2`: c₂ scaling factor used during the reel-out phase.
- `k_c2_high`: c₂ scaling factor for reel-out at high elevation angles.
- `k_c2_int`: c₂ scaling factor for the intermediate phases (`LOW_RIGHT`, `LOW_TURN`, `LOW_LEFT`).
- `k_ds`: influence of the depower angle on the steering sensitivity.
"""
@with_kw mutable struct FPCSettings @deftype Float64
    "period time"
    dt::Float64
    log_level::Int64         = 2
    prn::Bool                = false
    prn_ndi_gain::Bool       = false
    prn_est_psi_dot::Bool    = false
    prn_va::Bool             = false
    use_radius::Bool         = true
    use_chi::Bool            = true
    "reset the main integrator to the last estimated turn rate"
    reset_int1::Bool         = true
    "reset the integrator of the D part at the second time step"
    reset_int2::Bool         = false
    reset_int1_to_zero::Bool = true
    "if the root finder should start with zero"
    init_opt_to_zero::Bool   = false
    "P gain of the PID controller"
    p  = 20.0
    "I gain of the PID controller"
    i  = 1.2
    "D gain of the PID controller"
    d  = 10.0
    "additional factor for P, I and D"
    gain = 0.04
    c1 =  0.0612998898221 # was: 0.0786
    c2 =  1.22597628388   # was: 2.508
    "correction factor, used by the NDI block; increase k_c1, if the radius is too small; "
    k_c1 = 1.6
    "c2 for the reelout phase was: 7.0"
    k_c2 = 6.0
    "C2 for the reelout phase at high elevation angles was: 14.0"
    k_c2_high = 12.0
    "C2 for the intermediate phase LOW_RIGHT, LOW_TURN, LOW_LEFT"
    k_c2_int  =  0.6
    "influence of the depower angle on the steering sensitivity"
    k_ds = 2.0
end

StructTypes.StructType(::Type{FPCSettings}) = StructTypes.Mutable()

function update(fcs::FPCSettings)
    config_file = joinpath(get_data_path(), fpc_settings())
    if Sys.iswindows()
        config_file = replace(config_file, "/" => "\\")
    end
    if ! isfile(config_file)
        println("Warning: $config_file not found, using default settings.")
        return
    end
    dict = YAML.load_file(config_file)
    sec_dict = Dict(Symbol(k) => v for (k, v) in dict["fpc_settings"])
    StructTypes.constructfrom!(fcs, sec_dict)
end

function FPCSettings(update; dt)
    fcs = FPCSettings(; dt)
    if update
        KiteControllers.update(fcs)
    end
    fcs.dt = dt
    fcs
end