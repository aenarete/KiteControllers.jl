# activate the test environment if needed
using Pkg
if ! ("ControlPlots" ∈ keys(Pkg.project().dependencies))
    Pkg.activate(@__DIR__)
end
using ControlPlots, KiteControllers, Timers; tic()
using Printf: @sprintf
using Statistics: mean
using Test: @test, @testset
using KiteUtils: Settings, load_settings
using Printf: @sprintf

set::Settings = deepcopy(load_settings("system.yaml"))
fcs::FPCSettings = FPCSettings(dt=1/set.sample_freq)
DURATION = 100.0
SAMPLES = Int(DURATION / fcs.dt + 1)
TIME = range(0.0, DURATION, SAMPLES)

# Test the flight path controller against the simplified kite model as shown
# in diagram docs/flight_path_controller_test1.png .
u_d0 = 0.01 * set.depower_offset
u_d  = 0.01 * set.depowers[1]
fpc = FlightPathController(fcs; u_d0, u_d)
kite = KiteModel(fcs)
kite.omega = 0.08
v_a = 20.0
u_d = 0.19 # was: u_d_prime
psi_dot_set = 51.566
on_control_command(fpc; psi_dot_set=psi_dot_set)
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

p=plotx(TIME, PSI, BETA, PSI_DOT; 
      ylabels=["heading angle psi [°]","elevation β [°]", "psi_dot [rad/s]"], 
      fig = "test_fpc1")
display(p)

@testset "Flight path controller psi_dot tracking" begin
    start_idx = findfirst(t -> t >= 10.0, TIME)
    @test start_idx !== nothing

    if start_idx !== nothing
        psi_dot_slice = @view PSI_DOT[start_idx:end]
        psi_dot_avg = mean(psi_dot_slice)
        psi_dot_min = minimum(psi_dot_slice)
        psi_dot_max = maximum(psi_dot_slice)
        psi_dot_avg_deg = rad2deg(psi_dot_avg)
        psi_dot_min_deg = rad2deg(psi_dot_min)
        psi_dot_max_deg = rad2deg(psi_dot_max)
        psi_dot_error_pct = ((psi_dot_avg_deg / psi_dot_set) - 1) * 100
        variation_pct = ((psi_dot_max_deg - psi_dot_min_deg) / (2 * psi_dot_set)) * 100
        println("psi_dot stats for t>=10s:")
        println("avg:        $(@sprintf("%6.2f", psi_dot_avg_deg)) °/s")
        println("min:        $(@sprintf("%6.2f", psi_dot_min_deg)) °/s")
        println("max:        $(@sprintf("%6.2f", psi_dot_max_deg)) °/s")
        println("error:      $(@sprintf("%6.2f", psi_dot_error_pct)) %")
        println("variation: ±$(@sprintf("%6.2f", variation_pct)) %\n")
        @test abs(psi_dot_error_pct) < 5.0
        @test variation_pct < 25.0
    end
end
nothing

#     return TIME, PSI, BETA, PHI, PSI_DOT