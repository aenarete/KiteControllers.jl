using KiteControllers, KiteUtils, KiteModels
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
    dt = 0.05
    int1 = Integrator(dt)
    @test int1.output == 0.0
    @test int1.last_output == 0.0
    @test int1.i == 1.0
    int2 = Integrator(dt, 2)
    @test int2.output == 0.0
    @test int2.last_output == 0.0
    @test int2.i == 2.0
    int3 = Integrator(dt, 2, 2)
    @test int3.output == 2.0
    @test int3.last_output == 2.0
    @test int3.i == 2.0
    @test calc_output(int3, 0.5) == 2.05
    reset(int3, 1.1)
    @test int3.output == 1.1
    @test int3.last_output == 1.1
    @test int3.i == 2.0
    int3 = Integrator(dt, 2, 2)
    calc_output(int3, 0.5) == 2.05
    KiteControllers.on_timer(int3)
    @test int3.last_output == 2.05
end

@testset "WinchController" begin
    wcs = WCSettings()
    cvi = CalcVSetIn(wcs)
    force = wcs.f_low
    v_ro = calc_vro(wcs, force)
    @test v_ro ≈ 0
    v_ro = calc_vro(wcs, wcs.f_high)
    @test v_ro ≈ wcs.vf_max
    set_vset_pc(cvi, nothing, wcs.f_low)
    @test calc_output(cvi) ≈ 0
    set_vset_pc(cvi, nothing, wcs.f_high)
    @test calc_output(cvi) ≈ wcs.vf_max
    v_out_set = 1.0
    set_vset_pc(cvi, v_out_set)
    @test calc_output(cvi) ≈ wcs.vf_max
    for i in 1:15
        KiteControllers.on_timer(cvi)
    end
    @test calc_output(cvi) ≈ v_out_set
end

@testset "UnitDelay" begin
    ud = UnitDelay()
    for i in 1:3
        out=calc_output(ud, i)
        KiteControllers.on_timer(ud)
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
        KiteControllers.on_timer(rl)
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
        KiteControllers.on_timer(m2)
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
        KiteControllers.on_timer(m3)
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
        KiteControllers.on_timer(m3)
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
    set_inactive(pid1, false)
    set_v_set(pid1, -0.5)
    set_tracking(pid1, -0.5)
    set_inactive(pid1, false)
    set_v_set_in(pid1, 4.0)
    v_ro = 1.0
    set_v_act(pid1, v_ro)
    v_set_out = get_v_set_out(pid1)
    @test v_set_out ≈ -0.16
    set_v_act(pid1, 1.1)
    KiteControllers.on_timer(pid1)
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
    KiteControllers.on_timer(w)
    @test get_speed(w) ≈ 0.8988387476775817
    @test get_acc(w) ≈ 44.94193738387908
end

@testset "LowerForceController" begin
    wcs = WCSettings()
    lfc = LowerForceController(wcs)
    @test lfc.wcs.dt == 0.02
    KiteControllers._set(lfc)
    @test lfc.active
    lfc.v_act = 1.0
    lfc.force = 1.0
    KiteControllers._update_reset(lfc)
    @test ! lfc.active
    set_v_act(lfc, 0.0)
    @test lfc.v_act == 0.0
    set_force(lfc, 300)
    @test lfc.force == 300
    set_reset(lfc, true)
    @test lfc.reset
    set_v_sw(lfc, 2.2)
    @test lfc.v_sw == 2.2
    set_tracking(lfc, 3.3)
    @test lfc.tracking == 3.3
    lfc = LowerForceController(wcs)
    x = [0.1, 0.2]
    sat2_in, sat2_out, rate_out, int_in = KiteControllers.calc_sat2in_sat2out_rateout_intin(lfc, x)
    v_set_out = get_v_set_out(lfc)
    lfc.f_err = -2.0
    @test get_f_err(lfc) == -2.0
    lfc.f_set = 400.0
    @test get_f_set_low(lfc) == 0.0
    KiteControllers._set(lfc)
    @test get_f_set_low(lfc) == 400.0
    KiteControllers.on_timer(lfc)
    set_f_set(lfc, 330.0)
    @test lfc.f_set == 330.0
end

@testset "UpperForceController" begin
    wcs = WCSettings()
    ufc = UpperForceController(wcs)
    @test ufc.wcs.dt == 0.02
    KiteControllers._set(ufc)
    @test ufc.active
    ufc.v_act = -1.0
    ufc.force = 1.0
    KiteControllers._update_reset(ufc)
    @test ! ufc.active
    set_v_act(ufc, 0.0)
    @test ufc.v_act == 0.0
    set_force(ufc, 3500)
    @test ufc.force == 3500
    set_reset(ufc, true)
    @test ufc.reset
    set_v_sw(ufc, 2.2)
    @test ufc.v_sw == 2.2
    set_tracking(ufc, 3.3)
    @test ufc.tracking == 3.3
    ufc = UpperForceController(wcs)
    x = [0.1, 0.2, 0.3]
    sat2_in, sat2_out, rate_out, int_in, int2_in = KiteControllers.calc_sat2in_sat2out_rateout_intin(ufc, x)
    v_set_out = get_v_set_out(ufc)
    ufc.f_err = -2.0
    @test get_f_err(ufc) == -2.0
    ufc.f_set = 3300.0
    @test get_f_set_upper(ufc) == 0.0
    KiteControllers._set(ufc)
    @test get_f_set_upper(ufc) == 3300.0
    KiteControllers.on_timer(ufc)
    set_f_set(ufc, 330.0)
    @test ufc.f_set == 330.0
end

@testset "WinchController" begin
    wcs = WCSettings()
    wc = WinchController(wcs)
    v_act = 1.0
    force = 500.0
    f_low = wcs.f_low
    calc_v_set(wc, v_act, force, f_low)
    KiteControllers.on_timer(wc)
    @test get_state(wc) == 1 # wcsSpeedControl
    @test isnothing(get_set_force(wc))
end

@testset "FlightPathController" begin
    fcs = FPCSettings()
    fpc = FlightPathController(fcs) 
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
    turning, value = KiteControllers.get_state(fpc)
    @test ! turning
    @test value == [0.0, 0.0]
end

@testset "FPC_01" begin
    PARKING = false
    fcs = FPCSettings()
    fcs.dt = 0.02
    fpc = FlightPathController(fcs) 
    us = calc_steering(fpc, PARKING)
    @test us == 0.0
end

@testset "testNDI_01" begin
    # test nonlinear dynamic inversion
    fcs = FPCSettings()
    fcs.dt = 0.02
    fpc = FlightPathController(fcs)
    psi_dot = deg2rad(0.0)
    fpc.u_d_prime = 0.2
    fpc.va = 20.0
    fpc.psi = deg2rad(90.0)
    fpc.beta = deg2rad(45.0)
    u_s = KiteControllers.linearize(fpc, psi_dot)
    @test u_s ≈ 0.5963201316799875
end

@testset "FPC_02a" begin
    PARKING = false
    fcs = FPCSettings()
    fcs.dt = 0.02
    fpc = FlightPathController(fcs) 
    x = [0.1, 0]
    a,b,c,d,e = KiteControllers.calc_sat1in_sat1out_sat2in_sat2out(fpc, x)
    @test a ≈ 0.0004
    @test b ≈ 0.0004
    @test c ≈ 0.0007137043822911918
    @test d ≈ 0.0007137043822911918
    @test e ≈ 0.5
    x = [0.1, 0.1]
    a,b,c,d,e = KiteControllers.calc_sat1in_sat1out_sat2in_sat2out(fpc, x)
    # println(a,b,c,d,e)    # (0.0012, 0.0012, 0.002141113146873575, 0.002141113146873575, 1.5)
end

@testset "FPC_02" begin
    PARKING = false
    fcs = FPCSettings()
    fcs.dt = 0.02
    fpc = FlightPathController(fcs) 
    u_d = 0.24
    va = 24.0
    beta = deg2rad(70.0)
    psi = deg2rad(90.0)
    chi = psi
    omega = 5.0
    phi = 0.0
    on_est_sysstate(fpc, phi, beta, psi, chi, omega, va; u_d=u_d)
    @test fpc.u_d_max ≈ 0.422
    @test fpc.va_av ≈ 24.0
    KiteControllers.on_timer(fpc)
    us = calc_steering(fpc, PARKING)
    @test us == 0.99
end

@testset "FPC_03" begin
     # test navigate method
    fcs = FPCSettings()
    fcs.dt = 0.02
    fpc = FlightPathController(fcs) 
    phi_set = deg2rad(0)
    beta_set = deg2rad(50)
    fpc.attractor[1] = phi_set
    fpc.attractor[2] = beta_set
    fpc.phi = deg2rad(-45)
    fpc.beta = deg2rad(45)
    fpc.psi_dot_set = nothing
    KiteControllers.navigate(fpc)
    @test rad2deg(fpc.chi_set) ≈ -64.14299966054402
end

@testset "FPC_04" begin
     # test navigate method with active limit for delta_beta
    fcs = FPCSettings()
    fcs.dt = 0.02
    fpc = FlightPathController(fcs) 
    phi_set = deg2rad(0)
    beta_set = deg2rad(90)
    fpc.attractor[1] = phi_set
    fpc.attractor[2] = beta_set
    fpc.phi = deg2rad(45)
    fpc.beta = deg2rad(0)
    fpc.psi_dot_set = nothing
    KiteControllers.navigate(fpc)
    @test rad2deg(fpc.chi_set) ≈ 30.68205617643342
end

@testset "KiteModel" begin
    fcs = FPCSettings()
    fcs.dt = 0.02
    km = KiteControllers.KiteModel(fcs)
    @test km.omega == 0.08
    x = [0.0, 0]
    x0, x1, psi_dot = KiteControllers.calc_x0_x1_psi_dot(km, x)
    @test x0 ≈ 1.5707963267948966
    @test x1 ≈ 0.5759586531581288
    @test psi_dot ≈ 0.0
    x = [0.1, 0]
    x0, x1, psi_dot = KiteControllers.calc_x0_x1_psi_dot(km, x)
    @test x0 ≈ 1.571422282317272
    @test x1 ≈ 0.5759576516293584
    @test psi_dot ≈ 0.031297776118780624
    x = [0.1, 0.1]
    x0, x1, psi_dot = KiteControllers.calc_x0_x1_psi_dot(km, x)
    @test x0 ≈ 1.571419155146939
    @test x1 ≈ 0.5759576566328299
    @test psi_dot ≈ 0.031141417602125847
    KiteControllers.solve(km)
    @test km.psi_dot ≈ 0.262921024533129
    KiteControllers.on_timer(km)
end

@testset "SystemStateControl" begin
    wcs = WCSettings()
    fcs = FPCSettings()
    ssc = SystemStateControl(wcs, fcs)
    on_parking(ssc)
    @test ssc.state == ssParking
    on_autopilot(ssc)
    @test ssc.state == ssPowerProduction
    on_reelin(ssc)
    @test ssc.state == ssReelIn
    on_stop(ssc)
    @test ssc.state == ssManualOperation
    v_set = calc_v_set(ssc)
    @test isnothing(v_set)
    kcu = KCU(se())
    kps4 = KPS4(kcu)
    integrator = KiteModels.init_sim!(kps4, stiffness_factor=0.04)
    sys_state = SysState(kps4)
    on_new_systate(ssc, sys_state)
    v_set = calc_v_set(ssc)
    @test v_set > 0.0 && v_set < 0.1
    u_s = calc_steering(ssc)
    println(u_s)
end

@testset "FlightPathCalculator" begin
    fcs = FPCSettings()
    fpc = FlightPathController(fcs)
    fpca = FlightPathCalculator(fpc=fpc)
    vec=[1.0,2]
    res = KiteControllers.addy(vec, 0.5)
    @test res == [1.0, 2.5]
    res = KiteControllers.addxy(vec, 1.5, 1.0)
    @test res == [2.5, 3.0]
    KiteControllers.set_v_wind_gnd(fpca, 8.3)
    @test fpca._elevation_offset_p2 ==  4.0
    phi = deg2rad(0)
    beta = deg2rad(30)
    KiteControllers.set_azimuth_elevation(fpca, phi, beta)
    beta_set = 30.0
    KiteControllers._calc_beta_c1(fpca, beta_set)
    KiteControllers._calc_k2_k3(fpca, beta_set)
    KiteControllers._calc_t1(fpca, beta_set)
    KiteControllers.calc_p1(fpca, beta_set)
    KiteControllers.calc_p2(fpca, beta_set)
    KiteControllers.calc_p3(fpca)
    KiteControllers.calc_p4(fpca)
    KiteControllers.calc_t5(fpca, beta_set)
    KiteControllers.publish(fpca)
end
