# activate the test environment if needed
using Pkg
if ! ("ControlPlots" ∈ keys(Pkg.project().dependencies))
    using TestEnv; TestEnv.activate()
end
using KiteControllers, Timers, ControlPlots; tic()

# Test the flight path controller against the real 4point kite
# 1. park
# 2. fly towards attractor point P1

using KiteUtils
using KiteControllers, KiteModels, ControlPlots

kcu::KCU   = KCU(se())
kps4::KPS4 = KPS4(kcu)
wcs::WCSettings   = WCSettings();  wcs.dt = 1/se().sample_freq
fcs::FPCSettings  = FPCSettings(); fcs.dt = wcs.dt
fpps::FPPSettings = FPPSettings()
ssc::SystemStateControl = SystemStateControl(wcs, fcs, fpps)
dt::Float64 = wcs.dt

# the following values can be changed to match your interest
MAX_TIME::Float64 = 60
TIME_LAPSE_RATIO  = 1
MAX_ITER          = 80
SHOW_KITE         = false
# end of user parameter section #

function simulate(integrator)
    i=1
    sys_state = SysState(kps4)
    on_new_systate(ssc, sys_state)
    while true
        if i > 100
            depower = KiteControllers.get_depower(ssc)
            if depower < 0.22; depower = 0.22; end
            steering = calc_steering(ssc, 0)
            time = i * dt
            # disturbance
            if time < 20
                steering = 0.0
            elseif time < 21
                steering = 0.1
            end
            set_depower_steering(kps4.kcu, depower, steering)
        end  
        v_ro = 0.0
        KiteModels.next_step!(kps4, integrator, v_ro=v_ro, dt=dt)
        sys_state = SysState(kps4)
        on_new_systate(ssc, sys_state)
        if i*dt >= MAX_TIME break end
        i += 1
    end
    return 1
end

function test_parking(suppress_overshoot_factor = 3.0)
    global LAST_RES
    clear!(kps4)
    init_kcu(kcu, se())
    integrator = KiteModels.init_sim!(kps4, stiffness_factor=0.04)
    simulate(integrator)
end

# plotx(TIME, PSI, BETA, PSI_DOT; 
#       ylabels=["heading angle psi [°]","elevation β [°]", "psi_dot [rad/s]"], 
#       fig = "test_fpc1")

test_parking()