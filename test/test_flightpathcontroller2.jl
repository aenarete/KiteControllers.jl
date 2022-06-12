# activate the test environment if needed
using Pkg
if ! ("Plots" ∈ keys(Pkg.project().dependencies))
    using TestEnv; TestEnv.activate()
end
using KiteControllers, Timers; tic()

fcs = FPCSettings()
fcs.dt = 0.02
DURATION = 100.0
SAMPLES = Int(DURATION / fcs.dt + 1)
TIME = range(0.0, DURATION, SAMPLES)

# Test the flight path controller against the simplified kite model as shown
# in diagram docs/flight_path_controller_test1.png. Steer towards an
# attractor point.
using KiteControllers, Plots, BenchmarkTools
inspectdr()
InspectDR.defaults.xaxiscontrol_visible = false
InspectDR.defaults.pointdropmatrix = InspectDR.PDM_DEFAULTS[:never]
fpc = FlightPathController(fcs)
kite = KiteModel(fcs)
kite.omega = 0.08
v_a = 20.0
u_d_prime = 0.2
on_control_command(fpc, psi_dot_set=51.566)
PSI, BETA, PHI, PSI_DOT = zeros(SAMPLES), zeros(SAMPLES), zeros(SAMPLES), zeros(SAMPLES)
u_s = 0.0

#     attractor = np.zeros(2)
#     attractor[0] = radians(25.0) # phi_set
#     attractor[1] = radians(32.0) # beta_set
# #    attractor[0] = radians(0.0) # phi_set
# #    attractor[1] = radians(89.0) # beta_set
#     fpc.onNewControlCommand(attractor=attractor)
#     PSI, BETA, PHI, PSI_DOT = np.zeros(SAMPLES), np.zeros(SAMPLES), np.zeros(SAMPLES), np.zeros(SAMPLES)
#     ERR, K_PSI_OUT, K_U_OUT = np.zeros(SAMPLES), np.zeros(SAMPLES), np.zeros(SAMPLES)
#     INT_IN, INT_OUT, X0, U_S = np.zeros(SAMPLES), np.zeros(SAMPLES), np.zeros(SAMPLES), np.zeros(SAMPLES)
#     u_s = 0.0
#     with Timer() as t1:
#         for i in range(SAMPLES):
#             kite.setUS(u_s)
#             kite.setVA(v_a)
#             kite.solve()
#             psi_dot = kite.getPsiDot(); PSI_DOT[i] = degrees(psi_dot)
#             psi = kite.getPsi(); PSI[i] = degrees(psi)
#             beta = kite.getBeta(); BETA[i] = degrees(beta)
#             phi = kite.getPhi(); PHI[i] = degrees(phi)
#             chi = psi
#             omega = 0.0
#             fpc.onNewEstSysState(phi, beta, psi, chi, omega, v_a, u_d=u_d_prime, period_time=PERIOD_TIME)            
#             # fpc.onNewSystemState(phi, beta, psi, v_a, u_d_prime=u_d_prime)
#             u_s = fpc.calcSteering(False, PERIOD_TIME); U_S[i] = u_s
#             err = fpc.getErr(); ERR[i] = degrees(err)
#             K_PSI_OUT[i], K_U_OUT[i] = fpc.getKpsi_out(), fpc.getKu_out()
#             INT_IN[i], INT_OUT[i] = fpc.getIntIn(), fpc.getIntOut()
#             X0[i] = degrees(kite.getX0())
#             # print u_s
#             fpc.onTimer()
#             kite.onTimer()
#     print("time for executing the flight path control loop in us: ", form((t1.secs)  / SAMPLES * 1e6))
#     print("Passed test 05...")
#     return TIME, PSI, BETA, PHI, PSI_DOT, ERR, K_PSI_OUT, K_U_OUT, INT_IN, INT_OUT, X0, U_S