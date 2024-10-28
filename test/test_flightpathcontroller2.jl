# activate the test environment if needed
using Pkg
if ! ("ControlPlots" ∈ keys(Pkg.project().dependencies))
    using TestEnv; TestEnv.activate()
end
using KiteControllers, KiteUtils, ControlPlots, Timers; tic()

set = deepcopy(load_settings("system.yaml"))
set.sample_freq = 50

fcs::FPCSettings = FPCSettings(dt=1/set.sample_freq)
DURATION = 100.0
SAMPLES = Int(DURATION / fcs.dt + 1)
TIME = range(0.0, DURATION, SAMPLES)

# Test the flight path controller against the simplified kite model as shown
# in diagram docs/flight_path_controller_test1.png. Steer towards an
# attractor point.
u_d0 = 0.01 * set.depower_offset
u_d  = 0.01 * set.depower
fpc = FlightPathController(fcs; u_d0, u_d)
kite = KiteModel(fcs)
kite.omega = 0.08
v_a = 20.0
u_d_prime = 0.2
on_control_command(fpc, psi_dot_set=51.566)
PSI, BETA, PHI, PSI_DOT = zeros(SAMPLES), zeros(SAMPLES), zeros(SAMPLES), zeros(SAMPLES)
u_s = 0.0

attractor = zeros(2)
attractor[1] = deg2rad(25.0) # phi_set
attractor[2] = deg2rad(32.0) # beta_set
on_control_command(fpc, attractor=attractor)
PSI, BETA, PHI, PSI_DOT = zeros(SAMPLES), zeros(SAMPLES), zeros(SAMPLES), zeros(SAMPLES)
ERR, K_PSI_OUT, K_U_OUT = zeros(SAMPLES), zeros(SAMPLES), zeros(SAMPLES)
INT_IN, INT_OUT, X0, U_S = zeros(SAMPLES), zeros(SAMPLES), zeros(SAMPLES), zeros(SAMPLES)
u_s = 0.0
for i in 1:SAMPLES
    global u_s
    kite.u_s = u_s
    kite.v_a = v_a
    KiteControllers.solve(kite)
    kite.u_s = u_s
    kite.v_a = v_a
    KiteControllers.solve(kite)
    PSI_DOT[i] = kite.psi_dot
    PSI[i]     = rad2deg(kite.psi)
    BETA[i]    = rad2deg(kite.beta)
    PHI[i]     = rad2deg(kite.phi)
    chi = kite.psi
    omega = 0.0
    on_est_sysstate(fpc, kite.phi, kite.beta, kite.psi, chi, omega, v_a; u_d=u_d_prime)
    u_s = calc_steering(fpc, false); U_S[i] = u_s
    ERR[i] = rad2deg(fpc.err)
    K_PSI_OUT[i], K_U_OUT[i], = fpc.k_psi_out, fpc.k_u_out
#             INT_IN[i], INT_OUT[i] = fpc.getIntIn(), fpc.getIntOut()
#             X0[i] = degrees(kite.getX0())
    on_timer(fpc)
    on_timer(kite)
end
p=plotx(TIME, PSI, BETA, PSI_DOT; 
      ylabels=["heading angle psi [°]", "elevation β [°]", "psi_dot [rad/s]"], 
      fig="test_fpc2")
display(p)

plotxy(PHI, BETA, xlabel="azimuth ϕ [°]", ylabel="elevation β [°]", fig="test_fpc2_xy")
