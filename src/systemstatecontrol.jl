# Provides the component SystemStateControl. Implementation as specified in chapter five of
# the PhD thesis of Uwe Fechner.

#     Highest level state machine
#     Minimal set of states
#     ssParking, ssPowerProduction, ssReelIn
@with_kw mutable struct SystemStateControl @deftype Float64
    wc::WinchController
    fpp::FlightPathPlanner
    sys_state::Union{Nothing, SysState}    = nothing
    state::Observable(SystemState)[]       = ssParking
    tether_length::Union{Nothing, Float64} = nothing
    v_wind
end

function SystemStateControl(wcs::WCSettings, fcs::FPCSettings, fpps::FPPSettings; u_d0, u_d, v_wind)
    fpc = FlightPathController(fcs; u_d0, u_d)
    fpca = FlightPathCalculator(fpc, fpps)
    fpp = FlightPathPlanner(fpps, fpca)
    res = SystemStateControl(wc=WinchController(wcs), fpp=fpp, v_wind=v_wind)

    attractor = zeros(2)
    attractor[end] = deg2rad(80.0) # beta_set
    on_control_command(fpca.fpc, attractor=attractor)
    publish(fpca) # initialise the flight path calculator
    res
end

function on_parking(ssc::SystemStateControl, tether_length=nothing)
   ssc.tether_length = tether_length
   switch(ssc, ssParking)
end

function on_autopilot(ssc::SystemStateControl)
    switch(ssc, ssPowerProduction)
end

function on_winchcontrol(ssc::SystemStateControl)
    switch(ssc, ssWinchControl)
end

function on_reelin(ssc::SystemStateControl)
    switch(ssc, ssReelIn)
end

function on_stop(ssc::SystemStateControl)
    switch(ssc, ssManualOperation)
end

function on_new_systate(ssc::SystemStateControl, sys_state)
    ssc.sys_state=sys_state
end

function calc_v_set(ssc::SystemStateControl)
    if isnothing(ssc.sys_state)
        return nothing
    end
    if ssc.state == ssReelIn || ssc.fpp.fpca._sys_state == ssDepower
        f_low = ssc.wc.wcs.f_reelin
    else
        f_low = ssc.wc.wcs.f_low
    end
    force = ssc.sys_state.force[1]
    v_act = ssc.sys_state.v_reelout[1]
    if ssc.state in [ssParking, ssManualOperation]
        f_low = ssc.wc.wcs.f_low
        v_set = calc_v_set(ssc.wc, v_act, force, f_low, 0.0)
    else
        v_set = calc_v_set(ssc.wc, v_act, force, f_low)
    end
    on_timer(ssc.wc)
    v_set
end

function calc_steering(ssc::SystemStateControl, manual_steering = 0.0; heading = nothing)
    if isnothing(ssc.sys_state)
        return 0.0
    end
    phi  = ssc.sys_state.azimuth
    beta = ssc.sys_state.elevation
    if isnothing(heading)
        psi = ssc.sys_state.heading
    else
        psi = heading
    end
    v_a = ssc.sys_state.v_app
    chi = ssc.sys_state.course
    u_d = ssc.sys_state.depower
    length = ssc.sys_state.l_tether[1]
    height = ssc.sys_state.Z[end]
    set_azimuth_elevation(ssc.fpp.fpca, phi, beta)
    on_new_systate(ssc.fpp, phi, beta, psi, chi, v_a, u_d)
    if ssc.state == ssPowerProduction
        on_new_data(ssc.fpp, u_d, length, psi, height)
    end 
    if ssc.state in (ssParking, ssPowerProduction)
        u_s = calc_steering(ssc.fpp.fpca.fpc, (ssc.state == ssParking) )
    else 
        u_s = manual_steering
    end
    on_timer(ssc.fpp.fpca.fpc)
    u_s
end

function switch(ssc::SystemStateControl, state)
    if ssc.state == state && state != ssParking
        return
    end
    ssc.state = state
    # publish new system state
    if state == ssPowerProduction
        start(ssc.fpp, ssc.v_wind)
    end
    if state == ssParking
        on_new_system_state(ssc.fpp.fpca, state, true)
        _switch(ssc.fpp, PARKING)
    elseif state == ssReelIn
        on_new_system_state(ssc.fpp.fpca, state, true)
        _switch(ssc.fpp, DEPOWER)
    elseif state == ssWinchControl
        on_new_system_state(ssc.fpp.fpca, state, true)
        _switch(ssc.fpp, KiteControllers.POWER)
    end
    if ssc.fpp.fpps.log_level > 1
        println("New system state: ", Symbol(ssc.state))
    end
end

function get_depower(ssc::SystemStateControl)
    return depower[]
end
