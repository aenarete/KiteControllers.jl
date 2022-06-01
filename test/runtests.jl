using KiteControllers, KiteUtils
using Test

cd("..")
KiteUtils.set_data_path("") 

@testset "saturate" begin
    @test saturate( 10.0, -1,   1) == 1
    @test saturate(-10.0, -1,   1) == -1
    @test saturate( 10.0, -1.5, 1.5) == 1.5
    @test saturate( 0.5, -1.5, 1.5) == 0.5
end

# x=range(-4pi, 4pi, step=0.1)
# y=wrap2pi.(x)
# plot(x,y)

@testset "wrap2pi" begin
    @test wrap2pi(0) == 0
    @test wrap2pi(pi-0.001) == pi-0.001
    @test wrap2pi(-pi) == -pi
    @test wrap2pi(2pi) == 0
    @test wrap2pi(-2pi) == 0
end

@testset "Integrator" begin
    int1 = Integrator()
    @test int1.output == 0.0
    @test int1.last_output == 0.0
    @test int1.i == 1.0
    int2 = Integrator(0.05, 2)
    @test int2.output == 0.0
    @test int2.last_output == 0.0
    @test int2.i == 2.0
    int3 = Integrator(0.05, 2, 2)
    @test int3.output == 2.0
    @test int3.last_output == 2.0
    @test int3.i == 2.0
    @test calc_output(int3, 0.5) == 2.05
    reset(int3, 1.1)
    @test int3.output == 1.1
    @test int3.last_output == 1.1
    @test int3.i == 2.0
    int3 = Integrator(0.05, 2, 2)
    calc_output(int3, 0.5) == 2.05
    on_timer(int3)
    @test int3.last_output == 2.05
end

@testset "FlightPathController" begin
    fpc = FlightPathController() 
    on_control_command(fpc)
    phi = 0.0
    beta = 0.0
    psi = 0.0
    chi = 0.0
    omega = 0.0
    v_a = 0.0
    on_est_sysstate(fpc, phi, beta, psi, chi, omega, v_a, u_d=0.236)
    KiteControllers.navigate(fpc)
    psi_dot = 0.0
    KiteControllers.linearize(fpc, psi_dot)
    x = [0, 0]
    KiteControllers.calc_sat1in_sat1out_sat2in_sat2out(fpc, x)
    parking = false
    KiteControllers.calc_steering(fpc, parking)
end

@testset "WinchController" begin
    wcs = WCSettings()
    force = wcs.f_low
    v_ro = calc_vro(wcs, force; test=false)
    @test v_ro ≈ 0
    v_ro = calc_vro(wcs, wcs.f_high; test=false)
    @test v_ro ≈ wcs.vf_max

end

@testset "UnitDelay" begin
    ud = UnitDelay()
    for i in 1:3
        out=calc_output(ud, i)
        on_timer(ud)
        @test out == i-1
    end
    reset(ud)
    @test ud.last_input == 0
    @test ud.last_output == 0
end

@testset "RateLimiter" begin
    rl = RateLimiter(1.0, 0.5)
    input = [0,0,1,2,3,3,3,3,3,2,1,0,0,0,0,0]
    out = zeros(16)
    for i in 1:16
        out[i] = calc_output(rl, input[i])
        on_timer(rl)
        # println(input[i], " ", out[i])
    end
    @test out == [0,0,0.5,1,1.5,2,2.5,3,3,2.5,2,1.5,1,0.5,0,0]
end

@testset "Mixer_2CH" begin
    m2 = Mixer_2CH(1.0)
    x=[0,1,2,3,4,3,2,1,0,1,2]
    y=-x
    for i in 1:length(x)
        in1=x[i]
        in2=y[i]
    end
end
