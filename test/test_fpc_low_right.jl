# activate the test environment if needed
using Pkg
if ! ("ControlPlots" ∈ keys(Pkg.project().dependencies))
    Pkg.activate(@__DIR__)
end
using ControlPlots, KiteControllers, Timers; tic()

# Test the flight path controller against the real 4point kite
# 1. use initial condition from failure_low_right.arrow at 226.0s
# 2. fly towards attractor point P1
# 3. update u_d and v_reelout from logfile
# test fails if we allow v_ro > 0

using ControlPlots, KiteControllers, KiteModels, KiteViewers
using KiteUtils: Settings, load_settings

PROJECT="system_8000.yaml"

set::Settings = deepcopy(load_settings(PROJECT))
kcu::KCU   = KCU(set)
kps4::KPS4 = KPS4(kcu)
wcs::WCSettings   = WCSettings(true, dt = 1/set.sample_freq)
fcs::FPCSettings  = FPCSettings(true, dt = wcs.dt)
fpps::FPPSettings = FPPSettings(true)
dt::Float64 = wcs.dt
attractor=[55.73, 56.95]
# attractor=[0, 90] # zenith

u_d0 = 0.01 * set.depower_offset
u_d = 0.01 * set.depowers[1]
fpc = FlightPathController(fcs; u_d0, u_d)
fpca = FlightPathCalculator(fpc, fpps)
kite = KiteModel(fcs)
kite.omega = 0.08
u_d = 0.2

# the following values can be changed to match your interest
MAX_TIME::Float64 = 26.5
TIME_LAPSE_RATIO  = 1
MAX_ITER          = 80
SHOW_KITE         = true
fpc.fcs.log_level = 3
fpc.fcs.prn_ndi_gain = true
fpc.fcs.prn_va = true
T_START = 226.0
I_START = Int64(T_START/0.025+2)
# end of user parameter section #

viewer::Viewer3D = Viewer3D(SHOW_KITE)

log = load_log("failure_low_right.arrow2")
sl  = log.syslog
log_len = length(sl)

if I_START < 1 || I_START > log_len
    error("I_START=$(I_START) is outside log range 1:$(log_len)")
end

steps = Int64(MAX_TIME/dt)
particles = set.segments + 5
logger::Logger = Logger(particles, steps)

function simulate(integrator)
    i=1
    # Read initial controller state from individual columns to avoid
    # StructArray row access issues on undefined references.
    phi = sl.azimuth[I_START]
    beta = sl.elevation[I_START]
    psi = wrap2pi(sl.heading[I_START])
    chi = wrap2pi(sl.course[I_START])
    v_a = sl.v_app[I_START]
    last_phi_printed = nothing
    sys_state = SysState(kps4)
    println("on_control_command...")
    on_control_command(fpc; attractor=deg2rad.(attractor), intermediate=true)
    while true
        idx = I_START + i - 1
        if idx > log_len
            println("Stopping: reached end of input log at index $(log_len).")
            break
        end
        u_d = sl.depower[idx]
        v_ro = sl.v_reelout[idx]
        KiteControllers.set_azimuth_elevation(fpca, phi, beta)
        omega = fpca._omega
        # println("omega: $omega")
        if isnothing(last_phi_printed) || phi != last_phi_printed
            println("phi: ", rad2deg(phi))
            last_phi_printed = phi
        end
        on_est_sysstate(fpc, phi, beta, -psi, -chi, omega, v_a; u_d=u_d)
        steering = calc_steering(fpc, true)
        on_timer(fpc)
        set_depower_steering(kps4.kcu, u_d, steering)
        # v_ro = -1
        KiteModels.next_step!(kps4, integrator; set_speed=v_ro, dt=dt)
        sys_state = SysState(kps4)
        phi = sys_state.azimuth
        v_a = sys_state.v_app
        beta = sys_state.elevation
        psi = wrap2pi(sys_state.heading)
        chi = wrap2pi(sys_state.course)
        sys_state.var_06 = fpca.fpc.ndi_gain
        sys_state.var_07 = fpca.fpc.chi_set
        KiteViewers.update_system(viewer, sys_state; scale = 0.04/1.1, kite_scale=set.kite_scale)
        log!(logger, sys_state)
        sleep(dt/4)
        if i*dt >= MAX_TIME break end
        i += 1
    end
    return 1
end

function test_parking()
    clear!(kps4)
    KitePodModels.init_kcu!(kcu, set)
    integrator = KiteModels.init!(kps4, stiffness_factor=0.04)
    simulate(integrator)
end

test_parking()

# plot the result
KiteControllers.save_log(logger, "tmp")
lg = KiteControllers.load_log("tmp")
sl = lg.syslog
psi = rad2deg.(wrap2pi.(sl.heading))
plotx(sl.time, rad2deg.(sl.azimuth), rad2deg.(sl.elevation), sl.steering, sl.v_reelout, sl.var_06, rad2deg.(sl.var_07), -psi,
      ylabels=["azimuth", "elevation", "steering", "v_reelout", "ndi_gain", "chi_set", "-psi"], 
      fig="fpc_low_right")