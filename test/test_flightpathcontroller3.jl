# activate the test environment if needed
using Pkg
if ! ("ControlPlots" ∈ keys(Pkg.project().dependencies))
    using TestEnv; TestEnv.activate()
end
using KiteControllers, Timers, ControlPlots; tic()
if false; include("../src/flightpathcontroller.jl"); end
if false; include("../src/flightpathcalculator2.jl"); end

# Test the flight path controller against the real 4point kite
# 1. park
# 2. fly towards attractor point P1

using KiteUtils
using KiteControllers, KiteModels, KiteViewers, ControlPlots

PROJECT="system_8000.yaml"

set = deepcopy(se(PROJECT))
kcu::KCU   = KCU(set)
kps4::KPS4 = KPS4(kcu)
wcs::WCSettings   = WCSettings();  wcs.dt = 1/set.sample_freq
fcs::FPCSettings  = FPCSettings(); fcs.dt = wcs.dt
fpps::FPPSettings = FPPSettings()
dt::Float64 = wcs.dt
attractor=[55.73, 56.95]
# attractor=[0, 90] # zenith

fpc = FlightPathController(fcs)
fpca = FlightPathCalculator(fpc, fpps)
kite = KiteModel(fcs)
kite.omega = 0.08
u_d = 0.2

# the following values can be changed to match your interest
MAX_TIME::Float64 = 40
TIME_LAPSE_RATIO  = 1
MAX_ITER          = 80
SHOW_KITE         = true
fpc.fcs.log_level = 3
fpc.fcs.prn_ndi_gain = true
fpc.fcs.prn_va = true
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
                if time == 20.0
                    println("on_control_command...")
                    on_control_command(fpc; attractor=deg2rad.(attractor), intermediate=true)
                end
                phi  = sys_state.azimuth
                v_a = sys_state.v_app
                # chi = sys_state.course
                u_d = sys_state.depower
                beta = sys_state.elevation
                # psi = sys_state.heading
                psi = wrap2pi(sys_state.heading)
                chi = wrap2pi(sys_state.course)
                KiteControllers.set_azimuth_elevation(fpca, phi, beta)
                omega = fpca._omega
                # println("omega: $omega")
                on_est_sysstate(fpc, phi, beta, -psi, -chi, omega, v_a; u_d=u_d)
                steering = calc_steering(fpc, false)
            end
            on_timer(fpc)
            set_depower_steering(kps4.kcu, depower, steering)
        end  
        v_ro = 0.0
        KiteModels.next_step!(kps4, integrator, v_ro=v_ro, dt=dt)
        sys_state = SysState(kps4)
        KiteViewers.update_system(viewer, sys_state; scale = 0.04/1.1, kite_scale=set.kite_scale)
        sleep(dt/4)
        if i*dt >= MAX_TIME break end
        i += 1
    end
    return 1
end

function test_parking(suppress_overshoot_factor = 3.0)
    global LAST_RES
    clear!(kps4)
    init_kcu(kcu, set)
    integrator = KiteModels.init_sim!(kps4, stiffness_factor=0.04)
    simulate(integrator)
end

test_parking()