## this script shall:
## - run a simulation
## - create a log file
## - shall NOT use a GUI

using KiteControllers, KiteUtils, ControlPlots, NLsolve, LinearAlgebra
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
function residual(corr_vec=nothing; sim_time=460)
    if ! isnothing(corr_vec) 
        KiteControllers.save_corr(corr_vec)
    end
    set = deepcopy(KiteControllers.se())
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

    function play()
        on_parking(ssc)
        integrator=KiteModels.init_sim!(kps4, stiffness_factor=0.04)
        simulate(integrator)
    end

    play()
    println("Stopping...")
    on_stop(ssc)
    KiteControllers.save_log(logger, "tmp")
    lg = KiteControllers.load_log("tmp")
    ob = test_ob(lg, false)
    println("\n --> norm: ", norm(ob.corr_vec), "\n")
    ob.corr_vec
end

# function train()
#     initial = KiteControllers.load_corr()
#     sol = nlsolve(residual, initial; xtol=0.1, ftol=0.5, iterations=10)
#     sol.zero
# end

function train2()
    initial = KiteControllers.load_corr()
    for i in 1:5
        res = residual(initial)
        println("i: $(i), norm: $(norm(res))")
        initial .+= res
    end
    initial
end
