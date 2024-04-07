## this script shall:
## - run a simulation
## - create a log file
## - shall NOT use a GUI

using Pkg
if ! ("KiteViewers" ∈ keys(Pkg.project().dependencies))
    using TestEnv; TestEnv.activate()
end
using Timers; tic()

using KiteControllers, KiteModels, StatsBase

kcu::KCU   = KCU(se())
kps4::KPS4 = KPS4(kcu)

wcs = WCSettings(); update(wcs); wcs.dt = 1/se().sample_freq
fcs::FPCSettings = FPCSettings(); fcs.dt = wcs.dt
fpps::FPPSettings = FPPSettings()
ssc::SystemStateControl = SystemStateControl(wcs, fcs, fpps)
dt::Float64 = wcs.dt

function init_globals()
    global kcu, kps4, wcs, fcs, fpps, ssc
    kcu   = KCU(se())
    kps4 = KPS4(kcu)
    wcs = WCSettings(); update(wcs); wcs.dt = 1/se().sample_freq
    fcs = FPCSettings(); fcs.dt = wcs.dt
    fpps = FPPSettings()
    ssc = SystemStateControl(wcs, fcs, fpps)
end

# the following values can be changed to match your interest
MAX_TIME::Float64 = 460
TIME_LAPSE_RATIO  = 4
# end of user parameter section #

phi_set = 21.48
# on_control_command(ssc.fpp.fpca.fpc, attractor=[deg2rad(phi_set), deg2rad(51.88)])
# on_control_command(ssc.fpp.fpca.fpc, psi_dot_set=-23.763, radius=-4.35)

PARKING::Bool = false

steps = 0
LAST_I::Int64=0

function simulate(integrator, stopped=true)
    global LAST_I
    start_time_ns = time_ns()
    i=1
    j=0; k=0
    GC.gc()
    t_gc_tot = 0
    sys_state = SysState(kps4)
    on_new_systate(ssc, sys_state)
    while true
        if i > 100
            dp = KiteControllers.get_depower(ssc)
            if dp < 0.22 dp = 0.22 end
            steering = calc_steering(ssc)
            set_depower_steering(kps4.kcu, dp, steering)
        end
        if i == 200 && ! PARKING
            on_autopilot(ssc)
        end
        # execute winch controller
        v_ro = calc_v_set(ssc)
        #
        t_sim = @elapsed KiteModels.next_step!(kps4, integrator, v_ro=v_ro, dt=dt)
        sys_state = SysState(kps4)
        on_new_systate(ssc, sys_state)
        if mod(i, TIME_LAPSE_RATIO) == 0 
            wait_until(start_time_ns + 1e9*dt, always_sleep=true)          
            start_time_ns = time_ns()
            t_gc_tot = 0
        end
        i += 1
        if i*dt > MAX_TIME break end
    end
    return div(i, TIME_LAPSE_RATIO)
end

function play(stopped=false)
    global steps, kcu, kps4, wcs, fcs, fpps, ssc
    init_globals()
    on_parking(ssc)
    KiteModels.init_sim!(kps4, stiffness_factor=1.0)
    toc()
end

function autopilot()
    global PARKING
    PARKING = false
    on_autopilot(ssc)
end

function stop_()
    println("Stopping...")
    on_stop(ssc)
    clear!(kps4)
end

play(false)
stop_()
