## this script shall:
## - run a simulation
## - create a log file
## - shall NOT use a GUI

using Pkg
if ! ("KiteViewers" ∈ keys(Pkg.project().dependencies))
    using TestEnv; TestEnv.activate()
end
using Timers; tic()

using KiteControllers

kcu::KiteModels.KCU   = KiteModels.KCU(KiteControllers.se())
kps4::KiteModels.KPS4 = KiteModels.KPS4(kcu)

wcs = WCSettings(); update(wcs); wcs.dt = 1/KiteControllers.se().sample_freq
fcs::FPCSettings = FPCSettings(); fcs.dt = wcs.dt
fpps::FPPSettings = FPPSettings()
ssc::SystemStateControl = SystemStateControl(wcs, fcs, fpps)
dt::Float64 = wcs.dt

function init_globals()
    global kcu, kps4, wcs, fcs, fpps, ssc
    kcu   = KiteModels.KCU(KiteControllers.se())
    kps4 = KiteModels.KPS4(kcu)
    wcs = WCSettings(); update(wcs); wcs.dt = 1/KiteControllers.se().sample_freq
    fcs = FPCSettings(); fcs.dt = wcs.dt
    fpps = FPPSettings()
    ssc = SystemStateControl(wcs, fcs, fpps)
end

# the following values can be changed to match your interest
MAX_TIME::Float64 = 460
# end of user parameter section #

steps = 0

function simulate(integrator)
    i = 1
    sys_state = KiteModels.SysState(kps4)
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
        i += 1
        if i*dt > MAX_TIME break end
    end
    nothing
end

function play()
    init_globals()
    on_parking(ssc)
    integrator=KiteModels.init_sim!(kps4, stiffness_factor=1.0)
    toc()
    simulate(integrator)
end

play()
println("Stopping...")
on_stop(ssc)
KiteModels.clear!(kps4)
