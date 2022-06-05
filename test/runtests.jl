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
    cvi = CalcVSetIn(wcs)
    force = wcs.f_low
    v_ro = calc_vro(wcs, force; test=false)
    @test v_ro ≈ 0
    v_ro = calc_vro(wcs, wcs.f_high; test=false)
    @test v_ro ≈ wcs.vf_max
    set_vset_pc(cvi, nothing, wcs.f_low)
    @test calc_output(cvi) ≈ 0
    set_vset_pc(cvi, nothing, wcs.f_high)
    @test calc_output(cvi) ≈ wcs.vf_max
    v_out_set = 1.0
    set_vset_pc(cvi, v_out_set)
    @test calc_output(cvi) ≈ wcs.vf_max
    for i in 1:15
        on_timer(cvi)
    end
    @test calc_output(cvi) ≈ v_out_set
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
    m2 = Mixer_2CH(0.2, 1.0)
    x = ones(10)
    y = 2*x
    out = zeros(10)
    for i in 1:length(x)
        in1=x[i]
        in2=y[i]
        out[i] = calc_output(m2, x[i], y[i])
        select_b(m2, i > 2)
        on_timer(m2)
    end
    @test all(out .≈ [1.0, 1.0, 1.0, 1.2, 1.4, 1.6, 1.8, 2.0, 2.0, 2.0])
end

@testset "Mixer_3CH" begin
    m3 = Mixer_3CH(0.2, 1.0)
    x = ones(10)
    y = 2*x
    out = zeros(10)
    for i in 1:length(x)
        in1=x[i]
        in2=y[i]
        out[i] = calc_output(m3, x[i], y[i], 0)
        select_b(m3, i > 2)
        on_timer(m3)
    end
    @test all(out .≈ [1.0, 1.0, 1.0, 1.2, 1.4, 1.6, 1.8, 2.0, 2.0, 2.0])
    m3 = Mixer_3CH(0.2, 1.0)
    x = ones(10)
    y = 2*x
    out = zeros(10)
    for i in 1:length(x)
        in1=x[i]
        in2=y[i]
        out[i] = calc_output(m3, x[i], 0, y[i])
        select_c(m3, i > 2)
        on_timer(m3)
    end
    @test all(out .≈ [1.0, 1.0, 1.0, 1.2, 1.4, 1.6, 1.8, 2.0, 2.0, 2.0])    
end

@testset "SpeedController" begin
    wcs = WCSettings()
    sc = SpeedController(wcs)
    @test sc.wcs.dt == 0.02
    set_tracking(sc, 1.0)
    set_inactive(sc, true)
    set_inactive(sc, false)
    @test sc.integrator.output == 1.0
    @test sc.integrator.last_output == 1.0
    @test sc.limiter.output == 1.0
    @test sc.limiter.last_output == 1.0
    set_v_set_in(sc, 2.2)
    @test sc.v_set_in ≈ 2.2
    set_v_act(sc, 3.3)
    @test sc.v_act ≈ 3.3
    pid1 = SpeedController(wcs)
    set_v_set(pid1, -0.5)
    set_tracking(pid1, -0.5)
    set_inactive(pid1, false)
    set_v_set_in(pid1, 4.0)
    v_ro = 1.0
    set_v_act(pid1, v_ro)
    v_set_out = get_v_set_out(pid1)
    @test v_set_out ≈ -0.16
    set_v_act(pid1, 1.1)
    on_timer(pid1)
    v_set_out = get_v_set_out(pid1)
    @test get_v_error(pid1) ≈ 2.9
end

@testset "Winch" begin
    wcs = WCSettings()
    w = Winch(wcs)
    v_set = 4.0
    set_v_set(w, v_set)
    @test w.v_set == v_set
    force = 1000.0
    set_force(w, force)
    @test w.force == force
    on_timer(w)
    @test get_speed(w) ≈ 0.8988387476775817
    @test get_acc(w) ≈ 44.94193738387908
end


