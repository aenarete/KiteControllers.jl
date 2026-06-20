## this script provides the main functions 
## - residual()
## - train()
## The train() function trains the flight path planner for a specific
## wind speed and kite. It saves the learned correction vector by
## updating the `corr_vec:` entry in the project YAML configuration.

# activate the test environment if needed
using Pkg
if ! ("MakieControlPlots" ∈ keys(Pkg.project().dependencies))
    Pkg.activate(@__DIR__)
end
using KiteControllers, KiteModels
using KiteUtils: load_settings

PROJECT=read_project()

@enum SimError begin
    NoError
    TooLow
    TooHigh
    VelocityTooHigh
    VelocityTooLow
end

struct SimulationError
    code::SimError
    message::String
end

SimulationError() = SimulationError(NoError, "")

const tolerance  =   1.1 # allow 10% tolerance for velocity limits
const min_height =  6.0 # minimum height for simulation to be considered valid
const max_height = 600.0 # maximum height for simulation to be considered valid
MAX_NORM = 100.0        # maximum allowed norm for corr_vec
MAX_ITER = 70           # maximum number of training iterations

using MakieControlPlots, KiteControllers, LinearAlgebra, NonlinearSolve
import JLD2

global ssc::SystemStateControl

# Effective norm: exclude residuals for elements that are floor-clamped and still
# pushing lower — those crossings are physically unreachable and should not block
# convergence of the correctable crossings.
function effective_norm(res, vec, min_corr)
    s = 0.0
    for k = 1:min(length(res), length(vec)-1)
        if vec[k+1] <= min_corr + 1e-9 && res[k] < 0
            # floor-clamped and pushing lower — skip
        else
            s += res[k]^2
        end
    end
    for k = length(vec):length(res)
        s += res[k]^2
    end
    sqrt(s)
end

function test_ob(lg, plot=true)
    ob = KiteObserver()
    KiteControllers.observe!(ob, lg, FPPSettings(true).beta_set)
    if plot
        plotxy(ob.fig8, ob.elevation, xlabel="fig8", ylabel="elevation")
    else
        ob
    end
end

# run a simulation using a correction vector, return a log object
# If full_sim=true, run for the full set.sim_time (no early exit after cycle 3).
function residual(corr_vec=nothing; full_sim=false)
    l_in = 0
    if ! isnothing(corr_vec) 
        l_in=length(corr_vec)
    end
    set = deepcopy(load_settings(PROJECT))
    use_turbulence = 0.0
    set.use_turbulence = use_turbulence
    sim_time = set.sim_time
    kcu   = KiteModels.KCU(set)
    kps4 = KiteModels.KPS4(kcu)
    kps4.wm.v_min = 0.1
    KiteUtils.PROJECT = PROJECT
    wcs = WCSettings(true; dt = 1/set.sample_freq)
    wcs.dt = 1/set.sample_freq
    fcs = FPCSettings(true, dt=wcs.dt)
    fpps = FPPSettings(true)
    fpps.log_level = full_sim ? fpps.log_level : 1  # dots only during training; full output for full_sim
    u_d0 = 0.01 * set.depower_offset
    u_d  = 0.01 * set.depowers[1]
    ssc::SystemStateControl = SystemStateControl(wcs, fcs, fpps; u_d0, u_d, v_wind=set.v_wind)
    if ! isnothing(corr_vec)
        ssc.fpp.corr_vec = corr_vec
    end
    dt = wcs.dt
    steps = Int64(sim_time/dt)
    particles = set.segments + 5
    logger = KiteControllers.Logger(particles, steps)

    function simulate(integrator)
        i = 1
        error = SimulationError()
        sys_state = KiteModels.SysState(kps4)
        sys_state.e_mech = 0
        e_mech = 0
        sys_state.sys_state = Int16(ssc.fpp._state)
        on_new_systate(ssc, sys_state)
        KiteControllers.log!(logger, sys_state)
        while true
            if i > 100
                dp = KiteControllers.get_depower(ssc)
                if dp < 0.22 dp = 0.22 end
                heading = calc_heading(kps4; neg_azimuth=true, one_point=false)
                ssc.sys_state.heading = heading
                ssc.sys_state.azimuth = -calc_azimuth(kps4)
                local steering
                try
                    steering = -calc_steering(ssc)
                catch e
                    @warn "calc_steering crashed at t=$(round(i*dt, digits=1)) s: $e"
                    break
                end
                KiteModels.set_depower_steering(kps4.kcu, dp, steering)
            end
            if i == 200
                on_autopilot(ssc)
            end
            # execute winch controller
            local v_ro
            try
                v_ro = calc_v_set(ssc)
            catch e
                @warn "calc_v_set crashed at t=$(round(i*dt, digits=1)) s: $e"
                break
            end
            try
                KiteModels.next_step!(kps4, integrator; set_speed=v_ro, dt=dt)
            catch e
                @warn "Simulation crashed at t=$(round(i*dt, digits=1)) s: $e"
                break
            end
            sys_state = KiteModels.SysState(kps4)
            on_new_systate(ssc, sys_state)
            e_mech += (sys_state.winch_force[1] * sys_state.v_reelout[1])/3600*dt
            sys_state.e_mech = e_mech
            sys_state.sys_state = Int16(ssc.fpp._state)
            sys_state.cycle = ssc.fpp.fpca.cycle
            sys_state.fig_8 = ssc.fpp.fpca.fig8
            KiteControllers.log!(logger, sys_state)
            if i > 200
                if sys_state.Z[end] < min_height
                    error = SimulationError(TooLow, "Height $(round(sys_state.Z[end], digits=2)) m is below minimum $(min_height) m")
                    break
                end
                if sys_state.Z[end] > max_height
                    error = SimulationError(TooHigh, "Height $(round(sys_state.Z[end], digits=2)) m exceeds maximum $(max_height) m")
                    break
                end
                if sys_state.v_reelout[1] > tolerance * set.v_ro_max
                    error = SimulationError(VelocityTooHigh, "Reel-out speed $(round(sys_state.v_reelout[1], digits=3)) m/s exceeds limit $(round(tolerance * set.v_ro_max, digits=3)) m/s")
                    break
                end
                if sys_state.v_reelout[1] < tolerance * set.v_ro_min
                    error = SimulationError(VelocityTooLow, "Reel-out speed $(round(sys_state.v_reelout[1], digits=3)) m/s is below limit $(round(tolerance * set.v_ro_min, digits=3)) m/s")
                    break
                end
            end
            i += 1
            if i*dt > sim_time
                break 
            end
            if !full_sim && ssc.fpp.fpca.cycle >= 4
                break
            end
        end
        error
    end

    on_parking(ssc)
    integrator = KiteModels.init!(kps4; delta=set.delta, stiffness_factor=set.stiffness_factor)
    sim_error = simulate(integrator)
    println()  # end the progress-dot line
    on_stop(ssc)
    if sim_error.code != NoError
        @warn "Simulation ended with error: $(sim_error.message). Skipping result."
        return l_in > 0 ? fill(1000.0, l_in) : Float64[]
    end
    KiteControllers.save_log(logger, full_sim ? "last_sim_log" : "tmp"; path="output")
    lg = KiteControllers.load_log(full_sim ? "last_sim_log" : "tmp"; path="output")
    ob = test_ob(lg, false)
    test_ob(lg, true)
    min_safe_beta = 10.0
    min_corr = min_safe_beta - fpps.beta_set
    l_out = length(ob.corr_vec)
    if l_out == 0
        @warn "No flight path data collected (simulation may have crashed early). Skipping update."
        return l_in > 0 ? fill(1000.0, l_in) : Float64[]
    end
    if l_out < l_in
        for _ in 1:(l_in-l_out)
            push!(ob.corr_vec, 0)
        end
    end
    if l_in > 0
        result = ob.corr_vec[begin:l_in]
        n = norm(result)
        en = effective_norm(result, corr_vec, min_corr)
        println("\n --> norm: ", n, " (effective: ", en, ")\n")
        println("residual: ", round.(result, digits=3))
        return result
    end
    n = norm(ob.corr_vec)
    en = effective_norm(ob.corr_vec, ob.corr_vec, min_corr)
    println("\n --> norm: ", n, " (effective: ", en, ")\n")
    println("residual: ", round.(ob.corr_vec, digits=3))
    ob.corr_vec
end

function plot(;full_sim=false)
    if full_sim
        lg = KiteControllers.load_log("last_sim_log"; path="output")
        @info "Plotting last simulation log from output/last_sim_log.arrow"
    else
        lg = KiteControllers.load_log("tmp"; path="output")
        @info "Plotting current simulation log from output/tmp.arrow"
    end
    sl = lg.syslog
    fig_name = full_sim ? "azimuth_elevation_last" : "azimuth_elevation"
    display(MakieControlPlots.plotx(sl.time, rad2deg.(sl.azimuth), rad2deg.(sl.elevation);
            ylabels=["azimuth [°]", "elevation [°]"],
            xlabel="time [s]",
            fig=fig_name))
    nothing
end

function observe()
    lg = KiteControllers.load_log("tmp"; path="output")
    ob = KiteObserver()
    KiteControllers.observe!(ob, lg, FPPSettings(true).beta_set)
    println("corr_vec (length=$(length(ob.corr_vec))):")
    for v in ob.corr_vec
        println("  ", v)
    end
    nothing
end

function train(use_last=true; max_iter=MAX_ITER, norm_tol=1.0)
    local corr_vec
    if ! use_last
        try
            log = load_log("uncorrected")
            ob = KiteObserver()
            observe!(ob, log, FPPSettings(true).beta_set)
            corr_vec=ob.corr_vec
        catch
            corr_vec=residual()
        end
        if isempty(corr_vec)
            @warn "residual() returned an empty vector; skipping initial save_corr."
        else
            KiteControllers.save_corr(corr_vec)
        end
    end
    initial = FPPSettings(true).corr_vec
    beta_set = FPPSettings(true).beta_set
    min_safe_beta = 10.0  # minimum safe target elevation in degrees
    min_corr = min_safe_beta - beta_set  # floor for any correction element
    if norm(initial) > MAX_NORM
        @warn "Loaded corr_vec has large norm $(norm(initial)), resetting to zeros."
        initial = zeros(length(initial))
    end
    if norm(initial) < 1e-6
        @info "Loaded corr_vec is all zeros; training will start from zeros."
        # Do NOT fall back to FPPSettings().corr_vec — those defaults are calibrated
        # for a reference kite and will mislead training for other configurations.
    end
    best_corr_vec = deepcopy(initial)
    best_res  = zeros(length(initial))  # residual AT the best point (used for rollback direction)
    best_norm = Inf
    j = 0             # counts consecutive successful-but-no-improvement runs
    j_crash = 0       # counts consecutive crashes (safety limit)
    step_factor = 1.0
    correction_applied = false

    # Step multiplier schedule based on best_norm:
    #   far   (> 10):  0.5   — fast initial convergence
    #   mid   (5–10):  0.25  — moderate step as we close in
    #   close (2.5–5): 0.125 — careful near the solution
    #   fine  (< 2.5): 0.0625
    function step_mult(bn)
        bn > 10 ? 0.5 : bn > 5 ? 0.25 : bn > 2.5 ? 0.125 : 0.125
    end

    # Apply a scaled update from a residual vector to `initial`, shifting by +1 index.
    # Also syncs initial[1] (LOW_LEFT) to max(0, initial[2]) after each update:
    # crossing 1 elevation is controlled by LOW_LEFT, not just fig8-0-right, so both must
    # track together. The max(0,...) floor prevents negative LOW_LEFT (hydra20_426 case where
    # initial[2] = min_corr < 0 would otherwise crash the kite in the LOW phase).
    function apply_update!(vec, res_vec, sf, bn)
        sm = step_mult(bn)
        for k = 1:min(length(res_vec), length(vec)-1)
            vec[k+1] = max(min_corr, vec[k+1] + sf * sm * res_vec[k])
        end
        if length(vec) >= 2
            vec[1] = max(0.0, vec[2])
        end
    end

    for i in 1:max_iter
        res = residual(initial)
        en = effective_norm(res, initial, min_corr)
        n  = norm(res)
        if en < n - 1e-6
            println("i: $(i), norm: $(round(n, digits=4)) (effective: $(round(en, digits=4))), corr_vec[1:min(8, end)]=$(round.(initial[1:min(8, end)], digits=2))")
        else
            println("i: $(i), norm: $(round(n, digits=4)), corr_vec[1:min(8, end)]=$(round.(initial[1:min(8, end)], digits=2))")
        end
        crashed = length(res) > 0 && res[1] == 1000.0
        if !crashed && en < norm_tol
            println("Converged successfully using $i iterations!")
            best_corr_vec = deepcopy(initial)
            best_norm = en
            correction_applied = true
            break
        end
        if crashed
            # Roll back to best, halve the step, and re-apply from best_res.
            # best_res is the residual evaluated AT best_corr_vec, so the direction
            # is valid from that point regardless of how many stale iterations passed.
            j_crash += 1
            step_factor /= 2.0
            println("Crash detected: rolling back to best, step_factor=$(step_factor) (j_crash=$j_crash)")
            initial = deepcopy(best_corr_vec)
            apply_update!(initial, best_res, step_factor, best_norm)
            if j_crash > 8
                println("Too many consecutive crashes; giving up.")
                break
            end
        else
            j_crash = 0
            en = effective_norm(res, initial, min_corr)
            # Save best BEFORE updating initial (snapshot of the vector that produced best_norm).
            if best_norm > en
                best_norm = en
                best_corr_vec = deepcopy(initial)
                best_res = deepcopy(res)   # record residual AT the best point
                # Ramp step_factor back up gradually (×2, capped at 1.0).
                step_factor = min(step_factor * 2.0, 1.0)
                j = 0
                println("j: $(j), best_norm=$(round(best_norm, digits=4)), step_factor=$(step_factor)")
            else
                j += 1
                println("j: $j, step_factor=$(step_factor)")
                if j > 6
                    # Persistent non-improvement: roll back to best and halve the step.
                    # Use best_res (evaluated AT best_corr_vec) as the correction direction.
                    step_factor /= 2.0
                    j = 0
                    println("No improvement: step_factor reduced to $(step_factor); rolling back to best")
                    if step_factor < 1/32
                        println("Step factor too small ($(step_factor)); stopping.")
                        correction_applied = true
                        break
                    end
                    initial = deepcopy(best_corr_vec)
                    apply_update!(initial, best_res, step_factor, best_norm)
                    correction_applied = true
                    continue
                end
            end
            # res[k] = error at figure-8 crossing k → corrects initial[k+1].
            # initial[1] is the LOW_LEFT correction; not trained from crossing observations.
            apply_update!(initial, res, step_factor, best_norm)
            correction_applied = true
        end
    end
    if correction_applied && best_norm >= norm_tol
        println("Reached max_iter=$(max_iter) without full convergence. Best effective norm: $(round(best_norm, digits=4))")
    end
    last_nonzero = something(findlast(!iszero, best_corr_vec), length(best_corr_vec))
    best_corr_vec = best_corr_vec[1:last_nonzero]
    if best_norm < Inf && correction_applied
        KiteControllers.save_corr(best_corr_vec)
        println("Saved best corr_vec with norm=$(round(best_norm, digits=4)) (effective)")
    elseif !correction_applied && best_norm == Inf
        @warn "All simulations crashed; no corrections could be applied. Check project settings."
    elseif !correction_applied
        @info "Already converged before applying any correction; yaml file not updated."
    else
        @warn "Training produced no valid result. corr_vec not updated."
    end
    best_corr_vec
end

function plot_batch()
    lg = KiteControllers.load_log("batch-hydra20_600_TI0"; path="output")
    @info "Plotting batch log from output/batch-hydra20_600_TI0.arrow"
    sl = lg.syslog
    display(MakieControlPlots.plotx(sl.time, rad2deg.(sl.azimuth), rad2deg.(sl.elevation);
            ylabels=["azimuth [°]", "elevation [°]"],
            xlabel="time [s]",
            fig="azimuth_elevation_batch"))
    nothing
end

println("Available functions: plot(), plot_batch(), observe(), train(), residual()")
