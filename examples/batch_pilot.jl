# batch_pilot.jl
# Runs a full kite simulation without GUI or plots.
# Reads settings.yaml (and associated yaml config files) and saves the result to
#   output/batch-<timestamp>.arrow

using Pkg
if ! ("KiteControllers" ∈ keys(Pkg.project().dependencies))
    Pkg.activate(@__DIR__)
end
using Timers

using KiteControllers, KiteModels, Statistics
using Dates, LinearAlgebra, Printf

projects = ["hydra20_600.yml", "hydra20_426.yml", "hydra20_920.yml"]

@enum SimError begin
    NoError
    HitGround
    VelocityTooHigh
    VelocityTooLow
end

const tolerance = 1.1 # allow 10% tolerance for velocity limits, to avoid false positives due to numerical issues

function read_project(index::Int = 1)
    return projects[index]  
end

# ensure KiteUtils uses this project's data/ directory, regardless of cwd
set_data_path(joinpath(dirname(@__DIR__), "data"))

mutable struct KiteApp
    set::Settings
    max_time::Float64
    kcu::Union{KCU, Nothing}
    kps4::Union{KPS4, Nothing}
    wcs::Union{WCSettings, Nothing}
    fcs::Union{FPCSettings, Nothing}
    fpps::Union{FPPSettings, Nothing}
    ssc::Union{SystemStateControl, Nothing}
    logger::Union{Logger, Nothing}
    dt::Float64
    steps::Int64
    particles::Int64
    initialized::Bool
end



function init(app::KiteApp)
    app.kcu  = KCU(app.set)
    project  = KiteUtils.PROJECT
    app.kps4 = KPS4(app.kcu::KCU)
    KiteUtils.PROJECT = project

    app.wcs     = WCSettings(true; dt = 1/app.set.sample_freq)
    app.wcs.dt  = 1/app.set.sample_freq
    app.dt      = app.wcs.dt
    app.fcs     = FPCSettings(true; dt = app.dt)
    app.fcs.log_level = app.set.log_level
    app.fpps    = FPPSettings(true)
    app.fpps.log_level = app.set.log_level

    u_d0 = 0.01 * app.set.depower_offset
    u_d  = 0.01 * app.set.depowers[1]
    app.ssc = SystemStateControl(app.wcs, app.fcs, app.fpps;
                                 u_d0, u_d, v_wind = app.set.v_wind)

    app.steps     = Int64(app.max_time / app.dt)
    app.particles = app.set.segments + 5
    app.logger    = Logger(app.particles, app.steps)
    app.initialized = true
end

function simulate(app::KiteApp)
    on_parking(app.ssc::SystemStateControl)
    integrator = KiteModels.init!(app.kps4::KPS4; delta = 0.0009, stiffness_factor = 0.4)

    sys_state = SysState(app.kps4::KPS4)
    sys_state.e_mech   = 0
    sys_state.sys_state = Int16(app.ssc.fpp._state)

    e_mech        = 0.0
    last_vel      = [0.0, 0.0, 0.0]
    last_yaw      = 0.0
    last_yaw_rate = 0.0

    on_new_systate(app.ssc::SystemStateControl, sys_state)
    log!(app.logger::Logger, sys_state)

    println("Simulating $(app.max_time) s  (dt = $(app.dt) s, $(app.steps) steps) ...")

    error = NoError
    i = 1
    while i * app.dt <= app.max_time
        local v_ro

        # switch from parking to autopilot at step 200
        if i == 200
            on_autopilot(app.ssc::SystemStateControl)
        end

        if i > 100
            dp = KiteControllers.get_depower(app.ssc::SystemStateControl)
            if dp < 0.22; dp = 0.22; end
            heading = calc_heading(app.kps4::KPS4; neg_azimuth = true, one_point = false)
            app.ssc.sys_state.heading = heading
            app.ssc.sys_state.azimuth = -calc_azimuth(app.kps4::KPS4)
            steering = -calc_steering(app.ssc::SystemStateControl)
            set_depower_steering((app.kps4::KPS4).kcu, dp, steering)
        end

        v_ro = calc_v_set(app.ssc::SystemStateControl)
        KiteModels.next_step!(app.kps4::KPS4, integrator; set_speed = v_ro, dt = app.dt)

        sys_state  = SysState(app.kps4::KPS4)
        acc        = ((app.kps4::KPS4).vel_kite - last_vel) / app.dt
        last_vel   = deepcopy((app.kps4::KPS4).vel_kite)

        on_new_systate(app.ssc::SystemStateControl, sys_state)

        e_mech += (sys_state.winch_force[1] * sys_state.v_reelout[1]) / 3600 * app.dt
        sys_state.e_mech    = e_mech
        sys_state.sys_state = Int16(app.ssc.fpp._state)
        sys_state.cycle     = app.ssc.fpp.fpca.cycle
        sys_state.fig_8     = app.ssc.fpp.fpca.fig8
        sys_state.var_03    = get_state(app.ssc.wc)
        sys_state.var_04    = app.ssc.wc.lfc.f_set
        sys_state.var_05    = app.ssc.wc.lfc.v_set_out
        sys_state.var_06    = app.ssc.fpp.fpca.fpc.ndi_gain

        if isnothing(app.ssc.fpp.fpca.fpc.psi_dot_set)
            sys_state.var_07 = app.ssc.fpp.fpca.fpc.chi_set
            sys_state.var_10 = NaN
            sys_state.var_09 = NaN
        else
            sys_state.var_07 = NaN
            sys_state.var_09 = app.ssc.fpp.fpca.fpc.psi_dot_set
            sys_state.var_10 = app.ssc.fpp.fpca.fpc.est_psi_dot
        end

        sys_state.var_11 = app.ssc.fpp.fpca.fpc.est_chi_dot
        sys_state.var_12 = app.ssc.fpp.fpca.fpc.c2
        sys_state.acc    = norm(acc)

        if abs((sys_state.yaw - last_yaw) / app.dt) < 20.0
            sys_state.var_15 = (sys_state.yaw - last_yaw) / app.dt
        else
            sys_state.var_15 = last_yaw_rate
        end
        last_yaw      = sys_state.yaw
        last_yaw_rate = sys_state.var_15
        sys_state.var_16 = (app.kps4::KPS4).side_slip
        sys_state.var_08 = norm((app.kps4::KPS4).lift_force) / norm((app.kps4::KPS4).drag_force)

        log!(app.logger::Logger, sys_state)

        if sys_state.Z[end] < 0
            error = HitGround
            break
        end

        if sys_state.v_reelout[1] > tolerance * app.set.v_ro_max
            error = VelocityTooHigh
            break
        end

        if sys_state.v_reelout[1] < tolerance * app.set.v_ro_min
            error = VelocityTooLow
            break
        end

        if mod(i, Int64(round(200.0/app.dt))) == 0
            @printf("  t = %6.1f s  height = %6.1f m\n", i * app.dt, sys_state.Z[end])
        end

        i += 1
    end
    return i - 1, error
end

# ── run ────────────────────────────────────────────────────────────────────────
tic()
for project in projects
    println("Running project $project ...")
    app = KiteApp(deepcopy(load_settings(project)), 0.0,
                nothing, nothing, nothing, nothing, nothing, nothing, nothing,
                0.0, 0, 0, false)
    app.max_time = app.set.sim_time
    init(app)

    timestamp   = Dates.format(now(), "yyyy-mm-dd_HH-MM-SS")
    output_name = "batch-$(first(splitext(project)))-$timestamp"
    output_path = joinpath(dirname(@__DIR__), "output")

    steps, error = simulate(app)
    if error != NoError
        println("\nSimulation error: $error")
    else
        println("\nSimulation completed successfully (project = $project, steps = $steps)")
    end
    println("\nSaving log to output/$(output_name).arrow  ($(app.logger.index) entries) ...")
    save_log(app.logger, output_name; path = output_path)
    toc()
end
nothing
