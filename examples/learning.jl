## this script provides the main functions 
## - residual()
## - train()
## The train() function trains the flight path planner for a specific 
## wind speed and kite. It creates the file "data/corr_vec.jld2".

# activate the test environment if needed
using Pkg
if ! ("ControlPlots" ∈ keys(Pkg.project().dependencies))
    using TestEnv; TestEnv.activate()
    # pkg"add KiteModels#main"
end

function read_project()
    config_file = joinpath(get_data_path(), "gui.yaml")
    dict = YAML.load_file(config_file)
    dict["gui"]["project"]
end

PROJECT=read_project()

using KiteControllers, KiteUtils, ControlPlots, NonlinearSolve, LinearAlgebra
import JLD2

function test_ob(lg, plot=true)
    ob = KiteObserver()
    KiteControllers.observe!(ob, lg)
    if plot
        plotxy(ob.fig8, ob.elevation, xlabel="fig8", ylabel="elevation")
    else
        ob
    end
end

# run a simulation using a correction vector, return a log object
function residual(corr_vec=nothing; sim_time=500)
    l_in = 0
    if ! isnothing(corr_vec) 
        KiteControllers.save_corr(corr_vec)
        l_in=length(corr_vec)
    end
    set = deepcopy(KiteControllers.se(PROJECT))
    kcu   = KiteModels.KCU(set)
    kps4 = KiteModels.KPS4(kcu)

    wcs = WCSettings(); update(wcs); wcs.dt = 1/set.sample_freq
    fcs = FPCSettings(); fcs.dt = wcs.dt
    fpps = FPPSettings()
    ssc = SystemStateControl(wcs, fcs, fpps)
    if ! isnothing(corr_vec)
        ssc.fpp.corr_vec = corr_vec
    end
    dt = wcs.dt
    steps = Int64(sim_time/dt)
    particles = set.segments + 5
    logger = KiteControllers.Logger(particles, steps)

    function simulate(integrator)
        i = 1
        sys_state = KiteModels.SysState(kps4)
        sys_state.e_mech = 0
        e_mech = 0
        sys_state.sys_state = Int16(ssc.fpp._state)
        on_new_systate(ssc, sys_state)
        while true
            if i > 100
                dp = KiteControllers.get_depower(ssc)
                if dp < 0.22 dp = 0.22 end
                steering = calc_steering(ssc)
                KiteModels.set_depower_steering(kps4.kcu, dp, steering)
            end
            if i == 200
                on_autopilot(ssc)
            end
            # execute winch controller
            v_ro = calc_v_set(ssc)
            #
            t_sim = @elapsed KiteModels.next_step!(kps4, integrator, v_ro=v_ro, dt=dt)
            sys_state = KiteModels.SysState(kps4)
            on_new_systate(ssc, sys_state)
            e_mech += (sys_state.force * sys_state.v_reelout)/3600*dt
            sys_state.e_mech = e_mech
            sys_state.sys_state = Int16(ssc.fpp._state)
            sys_state.var_01 = ssc.fpp.fpca.cycle
            sys_state.var_02 = ssc.fpp.fpca.fig8
            if i > 10
                sys_state.t_sim = t_sim*1000
            end
            KiteControllers.log!(logger, sys_state)
            i += 1
            if i*dt > sim_time
                break 
            end
        end
        nothing
    end

    on_parking(ssc)
    integrator=KiteModels.init_sim!(kps4, stiffness_factor=0.04)
    simulate(integrator)
    on_stop(ssc)
    KiteControllers.save_log(logger, "tmp")
    lg = KiteControllers.load_log("tmp")
    ob = test_ob(lg, false)
    test_ob(lg, true)
    println("\n --> norm: ", norm(ob.corr_vec), "\n")
    l_out = length(ob.corr_vec)
    println("l_out: $l_out")
    if l_out < l_in
        for i in 1:(l_in-l_out)
            push!(ob.corr_vec, 0)
        end
    end
    if l_in > 0
        return ob.corr_vec[begin:l_in]
    end
    ob.corr_vec
end

function train(use_last=true; max_iter=40, norm_tol=1.0)
    local corr_vec
    if ! use_last
        try
            log = load_log("uncorrected")
            ob = KiteObserver()
            observe!(ob, log)
            corr_vec=ob.corr_vec
        catch
            corr_vec=residual()
        end
        KiteControllers.save_corr(corr_vec)
    end
    initial = KiteControllers.load_corr()
    last_norm=1000
    best_corr_vec = deepcopy(initial)
    best_norm = norm(best_corr_vec)
    j = 0
    for i in 1:max_iter
        res = residual(initial)
        println("i: $(i), norm: $(norm(res))")
        common_size=min(length(initial), length(res))
        for i = 1:common_size
            if best_norm > 5
                initial[i] += 0.5 * res[i]
            elseif best_norm > 2.5
                initial[i] += 0.25*res[i]
            else
                initial[i] += 0.125*best_norm*res[i]
            end
        end
        if best_norm > norm(res)
            best_norm = norm(res)
            best_corr_vec = deepcopy(initial)
            j = 0
            println("j: $(j), best_norm= $best_norm")
        else
            j+=1
            println("j: $j")
        end
        if norm(res) < norm_tol
            println("Converged successfully using $i iterations!")
            break
        end
        if j > 4
            println("Convergance failed!")
            println("Best norm: $best_norm")
            break
        end
        last_norm=norm(res)
    end
    KiteControllers.save_corr(best_corr_vec)
    best_corr_vec
end
