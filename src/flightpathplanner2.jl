# Provides the component FlightPathPlanner. Implementation as specified in chapter five of
# the PhD thesis of Uwe Fechner.

"""
    FlightPathPlanner

Top-level flight path planner as specified in chapter 5 of the PhD thesis of Uwe Fechner.

Manages the sequence of flight path phases (power, turns, intermediate, depower, parking)
by running a state machine whose transitions are evaluated on every call to
[`on_new_data`](@ref). It drives the [`FlightPathCalculator`](@ref), and thereby the
[`FlightPathController`](@ref), with appropriate attractor points or turn-rate commands.

# Fields
- `fpps::FPPSettings`: flight path planner settings (geometry, limits, PID gains).
- `fpca::FlightPathCalculator`: the associated flight path calculator.
- `corr_vec::Vector{Float64}`: elevation-angle correction look-up vector (loaded from `fpps.corr_vec`).
- `_state::FPPS`: current planner state (one of the `FPPS` enum values); starts at `INITIAL`.
- `delta_depower`: additional depower offset applied when generated power is too high.
- `const_dd`: depower interpolation coefficient `δ_d` (default `0.7`).
- `u_d_ro`: relative depower during reel-out (`0.01 * fpps.min_depower`).
- `u_d_ri`: relative depower during reel-in (`0.01 * fpps.max_depower`).
- `u_d_pa`: relative depower during parking (default `0.25`).
- `l_low`: lower tether-length limit for the reel-in/reel-out cycle [m].
- `l_up`: upper tether-length limit for the reel-in/reel-out cycle [m].
- `z_up`: maximum allowed kite height [m].
- `count`: general-purpose iteration counter.
- `finish::Bool`: flag set when the current reel-out cycle should finish after returning to centre.
- `last_phi`: azimuth angle from the previous time step [°], used for zero-crossing detection.
- `timeout`: counter incremented each call to `on_new_data`; reset on every state transition.
"""
@with_kw mutable struct FlightPathPlanner @deftype Float64
    fpps::FPPSettings
    fpca::FlightPathCalculator
    corr_vec::Vector{Float64} = load_corr()
    _state::FPPS = INITIAL
    delta_depower = 0    # this value must be increased, if the power is too high
    const_dd      = 0.7  # greek delta_depower
    u_d_ro        = 0.01 * fpps.min_depower
    u_d_ri        = 0.01 * fpps.max_depower
    u_d_pa        = 0.25 # parking depower
    l_low         = fpps.min_length
    l_up          = fpps.max_length
    z_up          = fpps.max_height
    count         = 0
    finish::Bool  = false
    last_phi::Float64 = 0
    timeout=0
end

function FlightPathPlanner(fpps::FPPSettings, fpca::FlightPathCalculator)
    FlightPathPlanner(fpps=fpps, fpca=fpca, corr_vec=fpps.corr_vec)
end

#  Call the related method of the flight path controller directly.
function _publish_fpc_command(fpp::FlightPathPlanner, _; attractor=nothing, psi_dot=nothing, radius=nothing, intermediate = false)
    if ! isnothing(psi_dot)
        psi_dot = rad2deg(psi_dot)
    end
    if ! isnothing(attractor)
        attractor=deg2rad.(attractor)
    end
    on_control_command(fpp.fpca.fpc, attractor=attractor, psi_dot_set=psi_dot, radius=radius, intermediate=intermediate)
    if fpp.fpps.log_level > 2
        println("New FPC command. Intermediate: ", intermediate)
            if isnothing(psi_dot) && !isnothing(attractor)
                @printf "New attractor point:  [%.2f,  %.2f]\n" rad2deg(attractor[begin]) rad2deg(attractor[begin+1])
            elseif !isnothing(psi_dot)
                if isnothing(radius)
                    @printf "New psi_dot_set: %.3f [°/s]\n" psi_dot
                else
                    @printf "New psi_dot_set: %.3f [°/s], radius: %.3f [°]\n" psi_dot radius
                end
            end
    end
end

#  Switch the state of the FPP. Execute all actions, that are needed when the new state is entered.
#  Return immediately, if the new state is equal to the old state.
function _switch(fpp::FlightPathPlanner, state)
    global depower
    if state == fpp._state
        return
    end
    fpp.timeout = 0
    psi_dot_turn = fpp.fpca._omega / fpp.fpca._radius # desired turn rate during the turns
    sys_state = fpp.fpca._sys_state
    # see Table 5.3
    if state == POWER
        depower = fpp.u_d_ro + fpp.delta_depower
        sys_state = ssPower
    elseif state == UPPER_TURN
        _publish_fpc_command(fpp, true, psi_dot = fpp.fpps.psi_dot_max, attractor=fpp.fpca._p1, intermediate=true)
        sys_state = ssIntermediate
        # see Table 5.4
    elseif state == LOW_RIGHT
        _publish_fpc_command(fpp, false, attractor = fpp.fpca._p1, intermediate = true)
        sys_state = ssIntermediate
    elseif state == LOW_TURN
        p2 = fpp.fpca._p2
        _publish_fpc_command(fpp, true, psi_dot = psi_dot_turn, radius=fpp.fpca._radius, attractor = p2, intermediate = true)
        sys_state = ssIntermediate
    elseif state == LOW_LEFT
        beta_set = corrected_elev(fpp.corr_vec,  fpp.fpps.beta_set)
        publish(fpp.fpca, beta_set)
        p2 = fpp.fpca._p2
        _publish_fpc_command(fpp, false, attractor = p2, intermediate = true)
        sys_state = ssIntermediate
    # see Table 5.5
    elseif state == TURN_LEFT
        ###fpps.beta_set
        elev_right, elev_left = corrected_elev(fpp.corr_vec, fpp.fpca.fig8, fpp.fpps.beta_set)
        beta_set = elev_right
        # println("TURN_LEFT: ", beta_set)
        publish(fpp.fpca, beta_set)

        radius = -fpp.fpca._radius
        _publish_fpc_command(fpp, true, psi_dot = -psi_dot_turn, radius=radius) #,  attractor = fpp.fpca._p3)
        sys_state = ssKiteReelOut
    elseif state == FLY_RIGHT
        _publish_fpc_command(fpp, false, attractor = fpp.fpca._p3)
        sys_state = ssKiteReelOut
    elseif state == TURN_RIGHT
        elev_right, elev_left = corrected_elev(fpp.corr_vec, fpp.fpca.fig8, fpp.fpps.beta_set)
        beta_set = elev_left
        # println("TURN_RIGHT: ", beta_set)
        publish(fpp.fpca, beta_set)

        radius = fpp.fpca._radius
        _publish_fpc_command(fpp, true, psi_dot = psi_dot_turn, radius=radius, attractor = fpp.fpca._p4)
        sys_state = ssKiteReelOut        
    elseif state == FLY_LEFT
        _publish_fpc_command(fpp, false, attractor = fpp.fpca._p4)
        sys_state = ssKiteReelOut
    # see Table 5.6
    elseif state == UP_TURN
        _publish_fpc_command(fpp, true, psi_dot = psi_dot_turn, radius=fpp.fpca._radius, attractor = fpp.fpca._zenith)
        sys_state = ssWaitUntil
    elseif state == UP_TURN_LEFT
        _publish_fpc_command(fpp, true, psi_dot = -psi_dot_turn, radius=-fpp.fpca._radius, attractor = fpp.fpca._zenith)
        sys_state = ssWaitUntil
    elseif state == UP_FLY_UP
        _publish_fpc_command(fpp, false, attractor = fpp.fpca._zenith)
        sys_state = ssWaitUntil
    elseif state == DEPOWER
        _publish_fpc_command(fpp, false, attractor = fpp.fpca._zenith)
        depower = fpp.u_d_ri # Table 5.3
        sys_state = ssDepower
    elseif state == PARKING
        _publish_fpc_command(fpp, false, attractor = fpp.fpca._zenith)
        depower = fpp.u_d_pa # Table 5.3
        sys_state = ssParking
    end

    if sys_state != fpp.fpca._sys_state
        on_new_system_state(fpp.fpca, sys_state, true)
        sleep(0.001)
    end
    if fpp.fpps.log_level > 2
        println("Switching to: ", state)
    end
    fpp._state = state
end

# Start automated power production; Precondition: The kite is parking at a high elevation angle.
function start(fpp::FlightPathPlanner, v_wind)
    if fpp.fpca._sys_state == ssManualOperation || fpp.fpca._sys_state == ssParking
        # see: Table 5.3
        _switch(fpp, KiteControllers.POWER)
    end
    set_v_wind_gnd(fpp.fpca, v_wind)
end

#  Check, if the new flight path planner is active. 
function is_active(fpp::FlightPathPlanner)
    fpp._state != INITIAL
end

# Return the state of the flight path planner as integer for logging.
function get_state(fpp::FlightPathPlanner)
    Int(fpp._state)
end

"""
    on_new_systate(fpp::FlightPathPlanner, phi, beta, heading, course, v_a, u_d)

Forward the current estimated kite system state to the [`FlightPathController`](@ref)
by calling `on_est_sysstate`. Sign conventions are applied before forwarding: azimuth and
heading angles are negated to match the controller's coordinate frame.

# Arguments
- `fpp`:     the `FlightPathPlanner` instance.
- `phi`:     azimuth angle of the kite in radians (positive to the left).
- `beta`:    elevation angle of the kite in radians.
- `heading`: kite heading angle `ψ` in radians, in the range `0 .. 2π`.
- `course`:  kite course angle `χ` in radians, in the range `0 .. 2π`.
- `v_a`:     apparent wind speed in m/s.
- `u_d`:     relative depower setting of the kite, in the range `0.0 .. 1.0`.
"""
function on_new_systate(fpp::FlightPathPlanner, phi, beta, heading, course, v_a, u_d)
    psi = wrap2pi(heading)
    chi = wrap2pi(course)
    on_est_sysstate(fpp.fpca.fpc, -phi, beta, -psi, -chi, fpp.fpca._omega, v_a, u_d=u_d)
end

"""
    on_new_data(fpp::FlightPathPlanner, depower, length, heading, height, _=0.0)

Evaluate the current kite state and trigger a flight-phase transition via `_switch` when
the transition conditions defined in Tables 5.3–5.6 of the PhD thesis are satisfied.

# Arguments
- `fpp`:     the `FlightPathPlanner` instance.
- `depower`: relative depower setting of the kite, in the range `0.0 .. 1.0`.
- `length`:  tether length in metres.
- `heading`: kite heading angle `ψ` in radians, in the range `0 .. 2π`.
- `height`:  kite height above ground in metres.

# State read from `fpp.fpca`
- `_phi`:   kite azimuth angle in degrees.
- `_beta`:  kite elevation angle in degrees.
- `_omega`: kite angular speed in degrees per second.
"""
function on_new_data(fpp::FlightPathPlanner, depower, length, heading, height, _=0.0)
    phi, psi  = fpp.fpca._phi, heading
    beta = fpp.fpca._beta
    phi_1 = fpp.fpca._t1[begin]
    phi_2 = fpp.fpca._phi_2
    phi_3 = fpp.fpca._phi_3
    state = fpp._state
    # see: Table 5.3, 5.4
    if state in [FLY_RIGHT, FLY_LEFT] && (sign(fpp.last_phi) != sign(phi))
        publish(fpp.fpca, fpp.fpps.beta_set)
        if state == FLY_RIGHT
            _publish_fpc_command(fpp, false, attractor = fpp.fpca._p3)
        else
            _publish_fpc_command(fpp, false, attractor = fpp.fpca._p4)
        end
    end
    fpp.last_phi = phi 
    if state == KiteControllers.POWER
        fpp.finish = false
        if (beta > fpp.fpca._beta_set + 25.0 + fpp.fpca._radius)
            if depower < fpp.u_d_ro + fpp.delta_depower + fpp.const_dd * (fpp.u_d_ri - fpp.u_d_ro - fpp.delta_depower)
                fpp.fpca.fig8 = -1
                fpp.fpca.cycle+=1
                fpp.fpca.high = false
                _switch(fpp, UPPER_TURN)
            end
        elseif depower < fpp.u_d_ro + fpp.delta_depower + fpp.const_dd * (fpp.u_d_ri - fpp.u_d_ro - fpp.delta_depower)
            fpp.fpca.fig8 = -1
            fpp.fpca.cycle+=1
            fpp.fpca.high = true
            _switch(fpp, FLY_LEFT)
        end
    elseif state == UPPER_TURN && psi > π && psi < deg2rad(fpp.fpps.heading_upper_turn)
        _switch(fpp, LOW_RIGHT)
    elseif state == LOW_RIGHT && phi < -phi_1
        fpp.fpca.fig8 += 1
        _switch(fpp, LOW_LEFT) # was LOW_TURN
    elseif state == LOW_TURN && psi < deg2rad(180.0 + fpp.fpps.heading_offset_int)          
        _switch(fpp, LOW_LEFT)
    elseif state == LOW_LEFT  && phi > -phi_2 # &&
        _switch(fpp, TURN_LEFT)
    # see: Table 5.5
    elseif state == FLY_LEFT  && phi > fpp.fpca._phi_sw 
        fpp.fpca.fig8 += 1            
        _switch(fpp, TURN_LEFT)
    elseif state == TURN_LEFT && (psi > deg2rad(180.0 - fpp.fpca._heading_offset) || fpp.timeout > fpp.fpps.timeout)
        _switch(fpp, FLY_RIGHT)
    elseif state == FLY_RIGHT && phi >= phi_3
        if ! fpp.finish
            fpp.finish = (length > fpp.l_up || height > fpp.z_up || fpp.fpca.fig8  == 6)
        end
    elseif state == FLY_RIGHT && fpp.finish && phi < phi_3
        _switch(fpp, UP_TURN_LEFT)
    elseif state == FLY_RIGHT && phi < -fpp.fpca._phi_sw # && 
        _switch(fpp, TURN_RIGHT)
    elseif state == TURN_RIGHT && (psi < deg2rad(180.0 + fpp.fpca._heading_offset) || fpp.timeout > fpp.fpps.timeout) 
        if fpp.fpps.log_level > 0
            println("timeout TURN_RIGHT: $(fpp.timeout)")
        end
        _switch(fpp, FLY_LEFT)
    elseif state == FLY_LEFT && phi <= -phi_3
        if ! fpp.finish
            fpp.finish = (length > fpp.l_up || height > fpp.z_up)
        end
    elseif state == FLY_LEFT && fpp.finish && phi > -phi_3
        _switch(fpp, UP_TURN)
    elseif state == UP_TURN && (psi > deg2rad(360.0 - fpp.fpps.heading_offset_up) || psi < deg2rad(fpp.fpps.heading_offset_up))
        _switch(fpp, UP_FLY_UP)
    elseif state == UP_TURN_LEFT && (psi > deg2rad(360.0 - fpp.fpps.heading_offset_up) || psi < deg2rad(fpp.fpps.heading_offset_up))
        _switch(fpp, UP_FLY_UP)
    elseif state == UP_FLY_UP && ((fpp.fpca._beta > fpp.fpca._beta_ri) || (height > fpp.z_up) || length > (fpp.l_up + 57.0))
        _switch(fpp, DEPOWER)
    # see: Table 5.3
    elseif state == DEPOWER && length < fpp.l_low
        fpp.fpca.fig8 = 0
        _switch(fpp, POWER)
    end
    fpp.timeout += 1
end
