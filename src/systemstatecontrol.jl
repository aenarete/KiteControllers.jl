# Provides the component SystemStateControl. Implementation as specified in chapter five of
# the PhD thesis of Uwe Fechner.

"""
    SystemStateControl

Highest-level state machine that coordinates the winch controller and the flight path
planner. It exposes a small set of commands (`on_autopilot`, `on_parking`, `on_reelin`,
`on_stop`, `on_winchcontrol`) that trigger state transitions, and two outputs per time
step: the winch speed set-point (`calc_v_set`) and the steering signal (`calc_steering`).

Construct via:
```julia
ssc = SystemStateControl(wcs::WCSettings, fcs::FPCSettings, fpps::FPPSettings;
                         u_d0, u_d, v_wind)
```
"""
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
    publish(fpca) # initialize the flight path calculator
    res
end

"""
    on_parking(ssc::SystemStateControl, tether_length=nothing)

Command the system to transition to the `ssParking` state.
Optionally pass the current `tether_length` [m] to allow a controlled approach.
"""
function on_parking(ssc::SystemStateControl, tether_length=nothing)
   ssc.tether_length = tether_length
   switch(ssc, ssParking)
end

"""
    on_autopilot(ssc::SystemStateControl)

Command the system to start automated power production (`ssPowerProduction` state).
Precondition: the kite must be parked at a sufficiently high elevation angle.
"""
function on_autopilot(ssc::SystemStateControl)
    switch(ssc, ssPowerProduction)
end

"""
    on_winchcontrol(ssc::SystemStateControl)

Switch to `ssWinchControl` state: winch is controlled automatically,
steering is performed manually.
"""
function on_winchcontrol(ssc::SystemStateControl)
    switch(ssc, ssWinchControl)
end

"""
    on_reelin(ssc::SystemStateControl)

Command the system to reel in the tether (`ssReelIn` state).
"""
function on_reelin(ssc::SystemStateControl)
    switch(ssc, ssReelIn)
end

"""
    on_stop(ssc::SystemStateControl)

Stop all automation and switch to `ssManualOperation`.
"""
function on_stop(ssc::SystemStateControl)
    switch(ssc, ssManualOperation)
end

"""
    on_new_systate(ssc::SystemStateControl, sys_state)

Feed the current `SysState` (from the kite model or measurement) into the
`SystemStateControl`. Must be called once per simulation time step before
calling `calc_v_set` and `calc_steering`.
"""
function on_new_systate(ssc::SystemStateControl, sys_state)
    ssc.sys_state=sys_state
end

"""
    calc_v_set(ssc::SystemStateControl)

Compute and return the winch speed set-point [m/s] for the current time step.
Returns `nothing` if no system state has been provided yet via `on_new_systate`.
"""
function calc_v_set(ssc::SystemStateControl)
    if isnothing(ssc.sys_state)
        return nothing
    end
    if ssc.state == ssReelIn || ssc.fpp.fpca._sys_state == ssDepower
        f_low = ssc.wc.wcs.f_reelin
    else
        f_low = ssc.wc.wcs.f_low
    end
    force = ssc.sys_state.winch_force[1]
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
        _switch(ssc.fpp, POWER)
    end
    if ssc.fpp.fpps.log_level > 1
        println("New system state: ", Symbol(ssc.state))
    end
end

"""
    get_depower(_ssc::SystemStateControl)

Return the current depower set-point (0..1) computed by the flight path planner.
"""
function get_depower(_ssc::SystemStateControl)
    return depower[]
end
