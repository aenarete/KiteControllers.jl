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
# in diagram docs/flight_path_controller_test1.png .
using KiteControllers, Plots, BenchmarkTools
inspectdr()
InspectDR.defaults.xaxiscontrol_visible = false
InspectDR.defaults.pointdropmatrix = InspectDR.PDM_DEFAULTS[:never]
fpc = FlightPathController(fcs)
kite = KiteModel(fcs)
kite.omega = 0.08
v_a = 20.0
u_d = 0.2 # was: u_d_prime
on_control_command(fpc, psi_dot_set=51.566)
PSI, BETA, PHI, PSI_DOT = zeros(SAMPLES), zeros(SAMPLES), zeros(SAMPLES), zeros(SAMPLES)
u_s = 0.0
for i in 1:SAMPLES
    global u_s
    kite.u_s = u_s
    kite.v_a = v_a
    KiteControllers.solve(kite)
    PSI_DOT[i] = kite.psi_dot
    PSI[i]     = rad2deg(kite.psi)
    BETA[i]    = rad2deg(kite.beta)
    PHI[i]     = rad2deg(kite.phi)
    chi = kite.psi
    omega = 5.0
    phi = 0.0
    on_est_sysstate(fpc, phi, kite.beta, kite.psi, chi, omega, v_a; u_d=u_d)
    u_s = calc_steering(fpc, false)
    on_timer(fpc)
    on_timer(kite)
end

p1=plot(TIME, PSI, label="heading angle psi [°]", width=2, xtickfontsize=12, ytickfontsize=12, legend=false)
pIDR = display(p1)           # Display with InspectDR and keep plot object
resize!(pIDR.wnd, 1200, 700) # Resize GTK window directly

p2=plot(TIME, BETA, label="elevation β [°]",     width=2, xtickfontsize=12, ytickfontsize=12, legendfontsize=12)
plot!(TIME, PHI,    label="azimuth ϕ   [°]",     width=2, xtickfontsize=12, ytickfontsize=12, legendfontsize=12)
pIDR2 = display(p2)           # Display with InspectDR and keep plot object
resize!(pIDR2.wnd, 1200, 700) # Resize GTK window directly

p3=plot(TIME, PSI_DOT, label="psi_dot [rad/s]",     width=2, xtickfontsize=12, ytickfontsize=12, legendfontsize=12)
pIDR3 = display(p3)           # Display with InspectDR and keep plot object
resize!(pIDR3.wnd, 1200, 700) # Resize GTK window directly

#     return TIME, PSI, BETA, PHI, PSI_DOT