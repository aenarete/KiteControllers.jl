# Provides the component FlightPathPlanner. Implementation as specified in chapter five of
# the PhD thesis of Uwe Fechner.

@with_kw mutable struct FlightPathPlanner @deftype Float64
    fpps::FPPSettings
    fpca::FlightPathCalculator
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
end

function FlightPathPlanner(fpps::FPPSettings, fpca::FlightPathCalculator)
    fpp = FlightPathPlanner(fpps=fpps, fpca=fpca)
end

# Start automated power production; Precondition: The kite is parking at a high elevation angle.
function start(fpp::FlightPathPlanner)
    if fpp.fpca._sys_state == ssManualOperation || fpp.fpca._sys_state == ssParking
        # see: Table 5.3
        _switch(fpp, POWER)
    end
    set_v_wind_gnd(fpp.fpca, se().v_wind)
end

#  Check, if the new flight path planner is active. 
function is_active(fpp::FlightPathPlanner)
    fpp._state != INITIAL
end

# Return the state of the flight path planner as integer for logging.
function get_state(fpp::FlightPathPlanner)
    Int(fpp._state)
end

# Parameters:
# phi:  the azimuth angle of the kite position in radian
# beta: the elevation angle of the kite position in radian
# psi:  heading of the kite in radian
# u_d:  relative depower of the kite (0..1)
function on_new_systate(fpp::FlightPathPlanner, phi, beta, heading, course, v_a, u_d)
    psi = wrap2pi(heading)
    chi = wrap2pi(course)
    on_est_sysstate(fpp.fpca.fpc, -phi, beta, -psi, -chi, fpp.fpca._omega, v_a, u_d=u_d)
end

#    on_new_data
# Determine, if a state change is need it and change it by calling the switch method, if neccessary.
#
# Parameters:
# depower: 0.0 .. 1.0
# length: tether length [m]
# heading: 0 .. 2 pi (psi)
# height: height [m]
# Inherited:
# fpp.fpca._phi:   azimuth in degrees
# fpp.fpca._beta:  elevation in degrees
# fpp.fpca._omega: angular speed in degrees per second
function on_new_data(fpp::FlightPathPlanner, depower, length, heading, height, time=0.0)
    phi, psi  = fpp.fpca._phi, heading
    beta = fpp.fpca._beta
    phi_1 = fpp.fpca._t1[begin]
    phi_2 = fpp.fpca._phi_2
    phi_3 = fpp.fpca._phi_3
    state = fpp._state
    # see: Table 5.3, 5.4
    if state == POWER
        fpp.finish = false
        if (beta > fpp.fpca._beta_set + 25.0 + fpp.fpca._radius)
            if depower < fpp.u_d_ro + fpp.delta_depower + fpp.const_dd * (fpp.u_d_ri - fpp.u_d_ro - fpp.delta_depower)
                fpp.fpca.fig8 = -1
                fpp.fpca.high = false
                _switch(fpp, UPPER_TURN)
            end
        elseif depower < fpp.u_d_ro + fpp.delta_depower + fpp.const_dd * (fpp.u_d_ri - fpp.u_d_ro - fpp.delta_depower)
            fpp.fpca.fig8 = -1
            fpp.fpca.high = true
            _switch(fpp, FLY_LEFT)
        end
    elseif state == UPPER_TURN && psi > π && psi < rad2deg(fpp.fpps.heading_upper_turn)
        _switch(fpp, LOW_RIGHT)
    elseif state == LOW_RIGHT && phi < -phi_1
        fpp.fpca.fig8 += 1
        _switch(fpp, LOW_TURN)
    elseif state == LOW_TURN && psi < rad2deg(180.0 + fpp.fpps.heading_offset_int)          
        _switch(fpp, LOW_LEFT)
    elseif state == LOW_LEFT  && phi > -phi_2 # &&
        _switch(fpp, TURN_LEFT)
    # see: Table 5.5
    elseif state == FLY_LEFT  && phi > fpp.fpca._phi_sw 
        fpp.fpca.fig8 += 1            
        _switch(fpp, TURN_LEFT)
    elseif state == TURN_LEFT && psi > deg2rad(180.0 - fpp.fpca._heading_offset)
        _switch(fpp, FLY_RIGHT)
    elseif state == FLY_RIGHT && phi >= phi_3
        if ! fpp.finish
            fpp.finish = (length > fpp.l_up || height > fpp.z_up)
        end
    elseif state == FLY_RIGHT && fpp.finish && phi < phi_3
        _switch(fpp, UP_TURN_LEFT)
    elseif state == FLY_RIGHT && phi < -fpp.fpca._phi_sw # && 
        _switch(fpp, TURN_RIGHT)
    elseif state == TURN_RIGHT && psi < deg2rad(180.0 + fpp.fpca._heading_offset) # && phi > -fpp._phi_sw - dphi
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
end

#  Call the related method of the flight path controller directly.
function _publish_fpc_command(fpp::FlightPathPlanner, turn; attractor=nothing, psi_dot=nothing, radius=nothing, intermediate = false)
    if ! isnothing(psi_dot)
        psi_dot = rad2deg(psi_dot)
    end
    if ! isnothing(attractor)
        attractor=deg2rad.(attractor)
    end
    on_control_command(fpp.fpca.fpc, attractor=attractor, psi_dot_set=psi_dot, radius=radius, intermediate=intermediate)
    if fpp.fpps.log_level > 2
        println("New FPC command. Intermediate: ", intermediate)
            if isnothing(psi_dot)
                @printf "New attractor point:  [%.2f,  %.2f]\n" rad2deg(attractor[begin]) rad2deg(attractor[begin+1])
            else
                if isnothing(radius)
                    @printf "New psi_dot_set: %.3f [°/s]\n" psi_dot
                else
                    @printf "New psi_dot_set: %.3f [°/s], radius: %.3f [°]\n" psi_dot radius
                end
            end
    end
end

#  Switch the state of the FPP. Execute all actions, that are needed when the new state is entered.
#  Return immidiately, if the new state is equal to the old state.
function _switch(fpp::FlightPathPlanner, state)
    global depower
    if state == fpp._state
        return
    end
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
        p2 = fpp.fpca._p2
        _publish_fpc_command(fpp, false, attractor = p2, intermediate = true)
        sys_state = ssIntermediate
    # see Table 5.5
    elseif state == TURN_LEFT
        radius = -fpp.fpca._radius
        _publish_fpc_command(fpp, true, psi_dot = -psi_dot_turn, radius=radius) #,  attractor = fpp.fpca._p3)
        sys_state = ssKiteReelOut
    elseif state == FLY_RIGHT
        _publish_fpc_command(fpp, false, attractor = fpp.fpca._p3)
        sys_state = ssKiteReelOut
    elseif state == TURN_RIGHT
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
    elseif state == UP_TURN_LEFT
        _publish_fpc_command(fpp, true, psi_dot = -psi_dot_turn, radius=-fpp.fpca._radius, attractor = fpp.fpca._zenith)
        sys_state = SystemState.ssWaitUntil
    elseif state == UP_FLY_UP
        _publish_fpc_command(fpp, false, attractor = fpp.fpca._zenith)
        sys_state = SystemState.ssWaitUntil
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
