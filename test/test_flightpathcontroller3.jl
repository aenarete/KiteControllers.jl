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
using KiteControllers, KiteModels, KiteViewers, ControlPlots

kcu::KCU   = KCU(se())
kps4::KPS4 = KPS4(kcu)
wcs::WCSettings   = WCSettings();  wcs.dt = 1/se().sample_freq
fcs::FPCSettings  = FPCSettings(); fcs.dt = wcs.dt
fpps::FPPSettings = FPPSettings()
# ssc::SystemStateControl = SystemStateControl(wcs, fcs, fpps)
dt::Float64 = wcs.dt

fpc = FlightPathController(fcs)
fpca = FlightPathCalculator(fpc, fpps)
kite = KiteModel(fcs)
kite.omega = 0.08
u_d = 0.2
on_control_command(fpc, psi_dot_set=51.566)


# the following values can be changed to match your interest
MAX_TIME::Float64 = 40
TIME_LAPSE_RATIO  = 1
MAX_ITER          = 80
SHOW_KITE         = true
# end of user parameter section #

viewer::Viewer3D = Viewer3D(SHOW_KITE)

function simulate(integrator)
    i=1
    sys_state = SysState(kps4)
    while true
        if i > 100
            depower = 0.22
            time = i * dt
            # first park
            if time < 20
                steering = 0.0
            # then steer towards P1
            else
                phi  = sys_state.azimuth
                v_a = sys_state.v_app
                chi = sys_state.course
                u_d = sys_state.depower
                beta = sys_state.elevation
                KiteControllers.set_azimuth_elevation(fpca, phi, beta)
                omega = fpca._omega
                # on_est_sysstate(fpc::FlightPathController, phi, beta, psi, chi, omega, va; u_d=nothing, u_d_prime=nothing)
                on_est_sysstate(fpc, phi, kite.beta, kite.psi, chi, omega, v_a; u_d=u_d)
                steering = calc_steering(fpc, false)
            end
            set_depower_steering(kps4.kcu, depower, steering)
        end  
        v_ro = 0.0
        KiteModels.next_step!(kps4, integrator, v_ro=v_ro, dt=dt)
        sys_state = SysState(kps4)
        # on_new_systate(ssc, sys_state)
        KiteViewers.update_system(viewer, sys_state; scale = 0.04/1.1, kite_scale=6.6)
        sleep(dt/4)
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