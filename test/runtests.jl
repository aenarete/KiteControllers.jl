using KiteControllers, KiteModels
using Test

cd("..")
KiteUtils.set_data_path("") 

@testset "FlightPathController" begin
    fcs = FPCSettings(dt=0.05)
    fpc = FlightPathController(fcs;  u_d0=0.01 * se().depower_offset, u_d=0.01 * se().depower) 
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
    fcs = FPCSettings(dt=0.05)
    fcs.dt = 0.02
    fpc = FlightPathController(fcs; u_d0=0.01 * se().depower_offset, u_d=0.01 * se().depower) 
    us = calc_steering(fpc, PARKING)
    @test us == 0.0
end

@testset "testNDI_01" begin
    # test nonlinear dynamic inversion
    fcs = FPCSettings(dt=0.05)
    fcs.dt = 0.02
    fpc = FlightPathController(fcs; u_d0=0.01 * se().depower_offset, u_d=0.01 * se().depower)
    psi_dot = deg2rad(0.0)
    fpc.u_d_prime = 0.2
    fpc.va = 20.0
    fpc.psi = deg2rad(90.0)
    fpc.beta = deg2rad(45.0)
    u_s = KiteControllers.linearize(fpc, psi_dot)
    @test u_s ≈ -0.31541575530041116
end

@testset "FPC_02a" begin
    PARKING = false
    fcs = FPCSettings(dt=0.05)
    fcs.dt = 0.02
    fpc = FlightPathController(fcs; u_d0=0.01 * se().depower_offset, u_d=0.01 * se().depower)
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
    fcs = FPCSettings(dt=0.05)
    fcs.dt = 0.02
    fpc = FlightPathController(fcs; u_d0=0.01 * se().depower_offset, u_d=0.01 * se().depower)
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
    fcs = FPCSettings(dt=0.05)
    fcs.dt = 0.02
    fpc = FlightPathController(fcs; u_d0=0.01 * se().depower_offset, u_d=0.01 * se().depower)
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
    fcs = FPCSettings(dt=0.05)
    fcs.dt = 0.02
    fpc = FlightPathController(fcs; u_d0=0.01 * se().depower_offset, u_d=0.01 * se().depower)
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
    fcs = FPCSettings(dt=0.05)
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
    wcs = WCSettings(dt=0.05)
    fcs = FPCSettings(dt=0.05)
    fpps = FPPSettings()
    ssc = SystemStateControl(wcs, fcs, fpps; u_d0=0.01 * se().depower_offset, u_d=0.01 * se().depower, v_wind = se().v_wind)
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
    integrator = KiteModels.init!(kps4, stiffness_factor=0.04)
    sys_state = SysState(kps4)
    on_new_systate(ssc, sys_state)
    v_set = calc_v_set(ssc)
    @test v_set >= 0.0 && v_set < 0.1
    u_s = calc_steering(ssc)
    println(u_s)
end

@testset "FlightPathCalculator" begin
    fcs = FPCSettings(dt=0.05)
    fpps = FPPSettings()
    fpc = FlightPathController(fcs; u_d0=0.01 * se().depower_offset, u_d=0.01 * se().depower)
    fpca = FlightPathCalculator(fpc, fpps)
    vec=[1.0,2]
    res = KiteControllers.addy(vec, 0.5)
    @test res == [1.0, 2.5]
    res = KiteControllers.addxy(vec, 1.5, 1.0)
    @test res == [2.5, 3.0]
    KiteControllers.set_v_wind_gnd(fpca, 8.2)
    KiteControllers.set_v_wind_gnd(fpca, 8.06)
    KiteControllers.set_v_wind_gnd(fpca, 8.3)
    KiteControllers.set_v_wind_gnd(fpca, 7.2)
    KiteControllers.set_v_wind_gnd(fpca, 6.2)
    KiteControllers.set_v_wind_gnd(fpca, 5.2)
    KiteControllers.set_v_wind_gnd(fpca, 3.7)
    KiteControllers.set_v_wind_gnd(fpca, 3.6)
    KiteControllers.set_v_wind_gnd(fpca, 8.3)
    # @test fpca._elevation_offset_p2 ==  11.0
    phi = deg2rad(0)
    beta = deg2rad(30)
    KiteControllers.set_azimuth_elevation(fpca, phi, beta)
    beta_set = 30.0
    KiteControllers._calc_beta_c1(fpca, beta_set)
    # KiteControllers._calc_k2_k3(fpca, beta_set)
    KiteControllers._calc_t1(fpca, beta_set)
    KiteControllers.calc_p1(fpca, beta_set)
    KiteControllers.calc_p2(fpca, beta_set)
    KiteControllers.calc_p3(fpca)
    KiteControllers.calc_p4(fpca)
    KiteControllers.calc_t5(fpca, beta_set)
    KiteControllers.publish(fpca)
end

@testset "FlightPathPlanner" begin
    fcs = FPCSettings(dt=0.05)
    fpps = FPPSettings()
    fpc = FlightPathController(fcs; u_d0=0.01 * se().depower_offset, u_d=0.01 * se().depower)
    fpca = FlightPathCalculator(fpc, fpps)
    fpp = FlightPathPlanner(fpps, fpca)
    @test fpp.u_d_ro == 0.22
    @test ! KiteControllers.is_active(fpp)
    @test KiteControllers.get_state(fpp) == 0
    phi = 0.0
    beta = 0.0
    heading = 0.0
    course = 0.0
    v_a = 10.0
    u_d = 0.25
    on_new_systate(fpp, phi, beta, heading, course, v_a, u_d)
    depower = u_d
    length = 150.0
    height = 100.0
    time = 0.0
    KiteControllers.on_new_data(fpp, depower, length, heading, height, time)
    fpp._state = POWER
    KiteControllers.on_new_data(fpp, depower, length, heading, height, time)
    fpp._state = LOW_LEFT
    KiteControllers.on_new_data(fpp, depower, length, heading, height, time)
    fpp._state = FLY_LEFT
    KiteControllers.on_new_data(fpp, depower, length, heading, height, time)
    fpp._state = TURN_LEFT
    KiteControllers.on_new_data(fpp, depower, length, heading, height, time)
    fpp._state = FLY_RIGHT
    KiteControllers.on_new_data(fpp, depower, length, heading, height, time)
    fpp._state = UP_TURN
    KiteControllers.on_new_data(fpp, depower, length, heading, height, time)
    fpp._state = UP_FLY_UP
    KiteControllers.on_new_data(fpp, depower, length, heading, height, time)
    fpp._state = DEPOWER
    KiteControllers.on_new_data(fpp, depower, length, heading, height, time)  
    fpp.count = 50
    KiteControllers.on_new_data(fpp, depower, length, heading, height, time)   
    KiteControllers.start(fpp, se().v_wind)
    KiteControllers._switch(fpp, POWER)
    @test fpp._state == POWER
    KiteControllers._switch(fpp, POWER)
    KiteControllers._switch(fpp, LOW_RIGHT)
    @test fpp._state == LOW_RIGHT
    KiteControllers._switch(fpp, LOW_TURN)
    @test fpp._state == LOW_TURN
    KiteControllers._switch(fpp, LOW_LEFT)
    @test fpp._state == LOW_LEFT
    KiteControllers._switch(fpp, FLY_RIGHT)
    @test fpp._state == FLY_RIGHT
    KiteControllers._switch(fpp, TURN_LEFT)
    @test fpp._state == TURN_LEFT
    KiteControllers._switch(fpp, TURN_RIGHT)
    @test fpp._state == TURN_RIGHT
    KiteControllers._switch(fpp, FLY_LEFT)
    @test fpp._state == FLY_LEFT
    KiteControllers._switch(fpp, UP_TURN)
    @test fpp._state == UP_TURN
    KiteControllers._switch(fpp, UP_TURN_LEFT)
    @test fpp._state == UP_TURN_LEFT
    KiteControllers._switch(fpp, UP_FLY_UP)
    @test fpp._state == UP_FLY_UP 
    KiteControllers._switch(fpp, PARKING)
    @test fpp._state == PARKING
end

include("aqua.jl")
