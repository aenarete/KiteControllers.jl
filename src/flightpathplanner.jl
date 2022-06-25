# Provides the components FlightPathPlanner, FlightPathCalculator and SystemStateControl. The main class for
# external use is FlightPathPlanner. Implementation as specified in chapter five of
# the PhD thesis of Uwe Fechner.

const depower = [0.0] # set value of the depower value, 0 .. 1

PRINT_EVERY_SECOND = true
PRINT = true

BETA_SET                 = 24.0 # for test 502
FIXED_ELEVATION          = false
CORRECT_RADIUS           = false
W_FIG                    = 36.0 # valid values: 36, 28

TRANS_FACTOR             = 2.3 # 1.5
DIRECT                   = false # skip intermediate target point
CLEAN                    = true  # do not use any correction

PSI_DOT_MAX = 3.0

ELEVATION_OFFSET         =  0.0
ELEVATION_OFFSET_40      =  0.0
ELEVATION_OFFSET_P1      =  0.0
ELEVATION_OFFSET_P2      = -4.0 # was: -4.4 
ELEVATION_OFFSET_P4_HIGH =  0.0
AZIMUTH_OFFSET_PHI_1     =  3.0
AZIMUTH_OFFSET_PHI_2     = -5.0 # was: -5.0 
AZIMUTH_OFFSET_PHI_2     =  0.0
ELEVATION_OFFSET_T2      =  3.5
ELEVATION_OFFSET_P3_ZERO = -2.4 # degrees; not used, when high elevation pattern is used
ELEVATION_OFFSET_P3_ZERO_HIGH = 0.0
ELEVATION_OFFSET_P3_ONE_HIGH  = 0.0
ELEVATION_OFFSET_P4_ZERO   = -2.0 # not used, when high elevation pattern is used
ELEVATION_OFFSET_P4_ONE   =   0.0
ELEVATION_OFFSET_P4_ONE_HIGH  =  0.0

if W_FIG >= 36.0
    HEADING_OFFSET_LOW  = 22.0 # degrees, before finishing the right and left turns
    HEADING_OFFSET_INT =  32.0 #54.0 # dito, for the turn around the intermediate point
elseif W_FIG <= 28.0
    HEADING_OFFSET_LOW =  20.0
    HEADING_OFFSET_INT =  62.0 #54.0 # dito, for the turn around the intermediate point
end
HEADING_OFFSET_HIGH = 54.0 # dito, for elevation angles > 47.5 degrees
HEADING_OFFSET_UP   = 60.0 # degrees, before finishing the up-turn
HEADING_UPPER_TURN =  360.0-25.0

DPHI_LOW    =  8.0           # azimuth offset for finishing the turns
DPHI_HIGH   = 10.0    
DELTA_PHI_1 =  6.0           # azimuth offset for finishing the turn around the intermediate point
DELTA_BETA = 1.0

function addy(vec, y)
    @SVector [vec[begin], vec[begin+1]+y]
end
  
function addxy(vec, x, y)
    @SVector [vec[begin]+x, vec[begin+1]+y]
end

@enum FPPS INITIAL=0 UPPER_TURN LOW_RIGHT LOW_TURN LOW_LEFT TURN_LEFT FLY_RIGHT TURN_RIGHT FLY_LEFT UP_TURN UP_TURN_LEFT UP_FLY_UP DEPOWER POWER PARKING
#     INITIAL                    = 0  # ssManualOperation
#     UPPER_TURN                 = 1  # ssIntermediate
#     LOW_RIGHT                  = 2  # ssIntermediate
#     LOW_TURN                   = 3  # ssIntermediate
#     LOW_LEFT                   = 4  # ssIntermediate
#     TURN_LEFT                  = 5  # ssKiteReelOut
#     FLY_RIGHT                  = 6  # ssKiteReelOut
#     TURN_RIGHT                 = 7  # ssKiteReelOut
#     FLY_LEFT                   = 8  # ssKiteReelOut
#     UP_TURN                    = 9  # ssWaitUntil
#     UP_TURN_LEFT               = 10
#     UP_FLY_UP                  = 11 # ssWaitUntil
#     DEPOWER                    = 12 # ssDepower
#     POWER                      = 13 # ssPower
#     PARKING                    = 14 # ssParking

# message AttractorPoint {
#    required double azimuth   = 1; // Angle in radians. Zero straight downwind. Positive direction clockwise seen
#                                   // from above. Valid range: -pi .. pi.
#                                   // Upwind is the direction the wind is coming from.
#    required double elevation = 2; // Angle in radians above the horizon. Valid range: -pi/2 to pi/2.
# }
# message PlannedFlightPath {
#     optional AttractorPoint p1         = 2;
#     optional AttractorPoint p2         = 3;
#     optional AttractorPoint p3         = 4;
#     optional AttractorPoint p4         = 5;
#     optional double phi_1              = 9;  // in degrees
#     optional double phi_2              = 10; // in degrees
#     optional double phi_3              = 11; // in degrees
#     optional double phi_sw             = 12; // in degrees
#     optional double beta_ri            = 13; // in degrees
# }

#     Component, that calculates the planned flight path as specified in the PhD thesis of
#     Uwe Fechner in chapter five.
#     Inputs:
#     a) The elevation angle at the end of the power phase (transition from ssPower to ssIntermediate)
#     b) azimuth and elevation angle of the kite at each time step for the calculation of omega
#     Outputs:
#     a) the planned flight path as defined in the message PlannedFlightPath (see above)
#     b) the angular speed of the kite (omega), projected on the unit sphere in degrees per second
#     A new flight path is calculated and published:
#     a) at the beginning of the reel-out phase (when the method onNewSystemState(ssIntermediate) is called )
#     b) when the set value of the elevation changes (call of the method publish(beta_set))
#     See also: docs/planned_fligh_path.png
@with_kw mutable struct FlightPathCalculator @deftype Float64
    fpc::FlightPathController
    _beta_min = 20.0 # minimal elevation angle of the center of the figure of eight
    _beta_max = 60.0 # maximal elevation angle of the center of the figure of eight

    _r_min =  3.0 # minimal turn radius in degrees
    _r_max =  4.5
    _radius = 4.5
    _w_fig = W_FIG # width of the figure of eight in degrees
    _dphi = 9.0 # correction for finishing the turns
    _phi_c3 = 0.0
    _beta_set = BETA_SET # average elevation angle during reel-out
    _beta_int = 68.5 # elevation angle at the start of beginning of the ssIntermediate
    _k = 0.0 # gradient of straight flight path sections, calculated value
    _k1 = 1.28
    _k4 = 0.175
    _k5 = 37.5
    _k6 = 0.45
    _delta_min = 10.0 # minimal attractor point distance in degrees
    _delta_phi = 0.0  # minimal attractor point distance in phi direction (calculated)
    _delta_beta = 0.0 # minimal attractor point distance in beta direction (calculated)
    _phi = 0.0    # kite position, azimuth
    _last_phi = _phi
    _beta = 66.0  # kite position, elevation
    _last_beta = _beta
    _omega = 0.0  # angular velocity of the kite in degrees per second
    _t1::MVector{2, Float64} = zeros(2)
    _t2::MVector{2, Float64} = zeros(2)
    _t3::MVector{2, Float64} = zeros(2)
    _t4::MVector{2, Float64} = zeros(2)
    _t5::MVector{2, Float64} = zeros(2) # point, where the upturn starts (end of flying fig. eight)
    _p1::MVector{2, Float64} = zeros(2)
    _p2::MVector{2, Float64} = zeros(2)
    _p3::MVector{2, Float64} = zeros(2)
    _p3_zero::MVector{2, Float64} = zeros(2)
    _p3_zero_high::MVector{2, Float64} = zeros(2)
    _p3_one_high::MVector{2, Float64} = zeros(2)
    _p4::MVector{2, Float64} = zeros(2)
    _p4_zero::MVector{2, Float64} = zeros(2)
    _p4_one::MVector{2, Float64} = zeros(2)
    _p4_one_high::MVector{2, Float64} = zeros(2)
    _zenith::MVector{2, Float64} = [0.0, 90] # desired elevation angle at zenith
    _phi_2 = 0.0
    _phi_3 = 0.0
    _phi_sw = 0.0
    _beta_ri = 0.0
    _heading_offset = HEADING_OFFSET_LOW
    _elevation_offset_p4 = 0.0
    _v_wind_gnd = 6.0 # ground wind speed at 6 m height
    _azimuth_offset_phi1 = AZIMUTH_OFFSET_PHI_1
    _elevation_offset_p2 = ELEVATION_OFFSET_P2
    _elevation_offset_t2 = ELEVATION_OFFSET_T2
    fig8 = 0 # number of the current figure of eight
    _sys_state::SystemState = ssManualOperation
    high::Bool = false
    elevation_offset = ELEVATION_OFFSET
end

function FlightPathCalculator(fpc::FlightPathController)
    FlightPathCalculator(fpc=fpc)
end

# Event handler for events, received from the GUI. The flight path planner
# must be in sync with the central system state.
function on_new_system_state(fpca::FlightPathCalculator, new_state::SystemState, internal = false)
    if fpca._sys_state == new_state
        return
    end
    fpca._sys_state = new_state
    if new_state == Int(ssPower)
        fpca._beta_int = fpca._beta
        # calculate and publish the flight path for the next cycle
        publish(fpca, fpca._beta_set)
    elseif new_state == Int(ssParking) && ! internal
        _switch(fpca, PARKING)
    end
end
            
# Set the ground wind speed at 6 m height
function set_v_wind_gnd(fpca::FlightPathCalculator, v_wind_gnd)
    fpca._v_wind_gnd = v_wind_gnd
    if v_wind_gnd > 8.2
        fpca._elevation_offset_p2 =  4.0
        fpca._elevation_offset_t2 = 10.5
        fpca._azimuth_offset_phi1 =  0.0               
    elseif v_wind_gnd > 8.06
        fpca._elevation_offset_p2 = 3.0
        fpca._elevation_offset_t2 = 9.5
        fpca._azimuth_offset_phi1 = 0.0
    elseif v_wind_gnd > 7.2
        fpca._elevation_offset_p2 = 1.7
        fpca._elevation_offset_t2 = 7.3
        fpca._azimuth_offset_phi1 = 0.0
    elseif v_wind_gnd > 6.2
        fpca._elevation_offset_p2 =  0.5
        fpca._elevation_offset_t2 =  4.2
        fpca._azimuth_offset_phi1 =  1.5
    elseif v_wind_gnd > 5.2
        fpca._elevation_offset_p2 = -4.0
        fpca._elevation_offset_t2 =  3.0
        fpca._azimuth_offset_phi1 =  3.0
    elseif v_wind_gnd > 3.7
        fpca._elevation_offset_p2 = -4.0 
        fpca._elevation_offset_t2 =  1.5
        fpca._azimuth_offset_phi1 =  3.0
    else
        fpca._elevation_offset_p2 = -6.0 
        fpca._elevation_offset_t2 = -0.5
        fpca._azimuth_offset_phi1 =  4.0
    end
end

# Set the kite position in spherical coordinates in the wind reference frame. 
function set_azimuth_elevation(fpca::FlightPathCalculator, phi, beta)
    period_time = fpca.fpc.fcs.dt
    fpca._phi = -rad2deg(phi)
    fpca._beta = rad2deg(beta)
    fpca._omega = sqrt(((fpca._beta - fpca._last_beta) / period_time)^2 
                            + ((fpca._phi - fpca._last_phi) / period_time)^2 * (cos(beta))^2)
    fpca._last_beta = fpca._beta
    fpca._last_phi = fpca._phi
end

# Calculate the elevation angle of the turning point as function of the set value of the
# average elevation angle during reel-out in degrees. 
function _calc_beta_c1(fpca::FlightPathCalculator, beta_set)
    fpca._radius = fpca._r_max - (fpca._r_max - fpca._r_min) * (beta_set - fpca._beta_min) /
                   (fpca._beta_max - fpca._beta_min)
    try
        @assert fpca._radius >= 2.8 # radius of the turns in degrees
        @assert fpca._radius <= 5.2
    catch
        println(fpca._r_max, fpca._r_min, beta_set, fpca._beta_min, fpca._beta_max, fpca._radius)
    end
    fpca._phi_c3 = fpca._w_fig/2.0 - fpca._radius
    delta_phi = fpca._radius^2 /fpca._phi_c3
    fpca._phi_sw = fpca._phi_c3 - fpca._radius^2 / fpca._phi_c3
    beta_sw = sqrt(fpca._radius^2 - (fpca._phi_sw - fpca._phi_c3)^2) + beta_set
    k = sqrt((fpca._phi_c3 - fpca._phi_sw) / fpca._phi_sw)
    beta_cw = beta_sw + k * (fpca._phi_c3 + delta_phi)
    beta_c1 = 0.5 * (fpca._beta_int + beta_cw)
    fpca._k = k
    beta_c1, k, beta_cw
end

function _calc_k2_k3(fpca::FlightPathCalculator, beta_set)
    if beta_set > 40.0
        k2 = beta_set - 40.0
    else
        k2 = 0.0
    end
    if fpca._r_min < 4.0
        k3 = 4.0 - fpca._r_min
    else
        k3 = 0.0
    end
    k2, k3
end

# Calculate azimuth and elevation of the point T1, where the first turn starts.
function _calc_t1(fpca::FlightPathCalculator, beta_set)
    k2, k3 = _calc_k2_k3(fpca, beta_set)
    beta_c1, k, beta_cw = _calc_beta_c1(fpca, beta_set)
    phi_c1 = (beta_c1 - beta_cw - 10.0 * fpca._k1 * (0.1 * fpca._radius)^1.2) / k 
    fpca._t1[begin+0] = (fpca._beta_int * k + phi_c1 - beta_c1 * k) / (k*k + 1.0)
    fpca._t1[begin+1] = fpca._beta_int - fpca._t1[begin+0] * k
    nothing
end

# Calculate azimuth and elevation of the intermediate attractor point P1.
function calc_p1(fpca::FlightPathCalculator, beta_set)
    _calc_t1(fpca, beta_set)
     fpca._delta_phi   = sqrt(fpca._delta_min^2 / (1.0 + fpca._k^2))
     fpca._delta_beta  = fpca._k * sqrt(fpca._delta_min^2 / (1.0 + fpca._k^2))
     fpca._p1[begin]   = fpca._t1[begin] + fpca._delta_phi
     fpca._p1[begin+1] = fpca._t1[begin+1] - fpca._delta_beta + ELEVATION_OFFSET_P1
     nothing
end

# Calculate azimuth and elevation of the second attractor point P2.
function calc_p2(fpca::FlightPathCalculator, beta_set)
    # Calculate azimuth and elevation of the point T2, where the figure-of-eight starts.
    println("===> calc_p2(fpca)")
    phi_sw, r         = fpca._phi_sw, fpca._radius
    fpca._t2[begin]   = -(fpca._phi_c3 - phi_sw) - fpca._phi_c3
    fpca._t2[begin+1] = sqrt(r * r - (phi_sw - fpca._phi_c3)^2) + beta_set
    fpca._p2[begin]   = fpca._t2[begin]   - fpca._delta_phi
    fpca._p2[begin+1] = fpca._t2[begin+1] - fpca._delta_beta
    nothing
end

# Calculate azimuth and elevation of the third attractor point P3.
function calc_p3(fpca::FlightPathCalculator)
    # Calculate azimuth and elevation of the point T3, where the right turn starts.
    fpca._t3[begin] = fpca._phi_sw
    fpca._t3[begin+1] = fpca._t2[begin+1]
    fpca._p3[begin] = fpca._t3[begin] + fpca._delta_phi
    fpca._p3_zero[begin] = fpca._p3[begin]
    fpca._p3_zero_high[begin] = fpca._p3[begin]
    fpca._p3_one_high[begin] = fpca._p3[begin]
    fpca._p3[begin+1] = fpca._t3[begin+1] + fpca._delta_beta
    fpca._p3_zero[begin+1] = fpca._p3[begin+1] + ELEVATION_OFFSET_P3_ZERO
    fpca._p3_zero_high[begin+1] = fpca._p3[begin+1] + ELEVATION_OFFSET_P3_ZERO_HIGH
    fpca._p3_one_high[begin+1] = fpca._p3[begin+1] + ELEVATION_OFFSET_P3_ONE_HIGH
    nothing
end

# Calculate azimuth and elevation of the forth attractor point P4.
function calc_p4(fpca::FlightPathCalculator)
    # Calculate azimuth and elevation of the point T4, where the left turn starts.
    fpca._t4[begin] = - fpca._phi_sw
    fpca._t4[begin+1] = fpca._t3[begin+1]
    fpca._p4[begin] = fpca._t4[begin] - fpca._delta_phi
    fpca._p4_one[begin] = fpca._p4[begin]
    fpca._p4_one_high[begin] = fpca._p4[begin]
    fpca._p4_zero[begin] = fpca._p4[begin]
    fpca._p4[begin+1] = (fpca._t4[begin+1] + fpca._delta_beta) + fpca._elevation_offset_p4
    fpca._p4_one[begin+1] = fpca._p4[begin+1] + ELEVATION_OFFSET_P4_ONE
    fpca._p4_one_high[begin+1] = fpca._p4[begin+1] + ELEVATION_OFFSET_P4_ONE_HIGH
    fpca._p4_zero[begin+1] = fpca._p4[begin+1] + ELEVATION_OFFSET_P4_ZERO
    nothing
end

# Calculate azimuth and elevation of the point T5, where the up-turn starts.
function calc_t5(fpca::FlightPathCalculator, beta_set)
    r, k = fpca._radius, fpca._k
    fpca._t5[begin] = r - sqrt(k * k * r * r / (k * k + 1.0))
    fpca._t5[begin+1] = beta_set - k * fpca._t5[begin]
    nothing
end

# Calculate and publish the planned flight path. Must be called each time when
# the winch controller calculates a new set value for the elevation, but also, when beta_int
# changes (at the beginning of each intermediate phase).
function publish(fpca::FlightPathCalculator, beta_set = BETA_SET)
    if FIXED_ELEVATION
        beta_set = BETA_SET
    end
    if beta_set > 30.0
        fpca._w_fig = W_FIG + 10.0
    end
    if beta_set > 52.0
        fpca._w_fig = W_FIG +  5.0
    else
        fpca._w_fig = W_FIG
    end
    if beta_set >= 47.5
        fpca._heading_offset = HEADING_OFFSET_HIGH
        fpca._dphi = DPHI_HIGH
        fpca._elevation_offset_p4 = ELEVATION_OFFSET_P4_HIGH
    else
        if fpca._v_wind_gnd > 9.2
            fpca._heading_offset = HEADING_OFFSET_LOW + 4.0
        else
            fpca._heading_offset = HEADING_OFFSET_LOW
        end
        fpca._dphi = DPHI_LOW
        fpca._elevation_offset_p4 = 0.0
    end
    rel_elevation = (beta_set - 20.0) / 20.0
    fpca._beta_set = beta_set
    fpca.elevation_offset = ELEVATION_OFFSET + rel_elevation * ELEVATION_OFFSET_40
    beta_set += fpca.elevation_offset

    calc_p1(fpca, beta_set)
    calc_p2(fpca, beta_set)
    calc_p3(fpca)
    calc_p4(fpca)
    calc_t5(fpca, beta_set)
    if TRANS_FACTOR >= 1.5
        fpca._t1[begin] /= TRANS_FACTOR
    end
    fpca._p1[begin] /= TRANS_FACTOR
    phi_1 = fpca._t1[begin]
    fpca._phi_2 = fpca._t2[begin]
    fpca._phi_3 = fpca._t5[begin]
    fpca._beta_ri = fpca._k5 + fpca._k6 * beta_set + ELEVATION_OFFSET
#   fpca.pub.publishPlannedFlightPath(fpca._p1, fpca._p2, fpca._p3, fpca._p4, fpca._t1[begin], fpca._t2[begin], \
#                                     fpca._t5[begin], fpca._phi_sw, fpca._beta_ri)
    nothing
end

# message AttractorPoint {
#    required double azimuth   = 1; // Angle in radians. Zero straight downwind. Positive direction clockwise seen
#                                   // from above. Valid range: -pi .. pi.
#                                   // Upwind is the direction the wind is coming from.
#    required double elevation = 2; // Angle in radians above the horizon. Valid range: -pi/2 to pi/2.
# }

# message FPC_Command {
#     required bool turn                 = 2;  // if true, than a turn rate must be given, otherwise an attractor 
#                                                 point
#     optional double psi_dot            = 3;  // desired turn rate in degrees per second
#     optional AttractorPoint attractor  = 4;  // the kite should fly towards this point
# }

#     Component, that implements the state machine as described in the PhD thesis of Uwe Fechner, chapter
#     five. It uses the pre-calculated flight path together
#     with incoming kite state data to calculate the FPC_Command messages, if needed.

# Inputs:
# a) the inherited flight path and angular kite speed omega (on the unit circle)
# b) the actual depower value of the kite
# c) the actual reel-out length of the tether
# d) the actual orientation of the kite: the heading angle
# e) the height of the kite

#     Output:
#     FPC_Command messages as defined above.
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
    # println("phi_1, phi: ", phi_1, ", ", phi)
    phi_2 = fpp.fpca._phi_2
    phi_3 = fpp.fpca._phi_3
    state = fpp._state
    if fpp.fpca.fig8 == 0
        dphi = fpp.fpca._dphi + 5.0 
    elseif fpp.fpca.fig8 <= 2
        dphi = fpp.fpca._dphi + 3.0
    else
        dphi = fpp.fpca._dphi + 2.0
    end
    # see: Table 5.3, 5.4
    if state == POWER
        fpp.finish = false
        # use the intermediate point only if we have to fly down more than 20 degrees
        delta_beta = beta - fpp.fpca._beta_set - fpp.fpca._radius
        if (beta > fpp.fpca._beta_set + 25.0 + fpp.fpca._radius) && ! DIRECT
            if depower < fpp.u_d_ro + fpp.delta_depower + fpp.const_dd * (fpp.u_d_ri - fpp.u_d_ro - fpp.delta_depower)
                fpp.fpca.fig8 = -1
                fpp.fpca.high = false
                _switch(fpp, UPPER_TURN)
            end
        elseif depower < fpp.u_d_ro + fpp.delta_depower + fpp.const_dd * (fpp.u_d_ri - fpp.u_d_ro - fpp.delta_depower)
            fpp.fpca.fig8 = -1
            fpp.fpca.high = true
            _switch(fpp, FLY_LEFT, delta_beta)
        end
    elseif state == UPPER_TURN && psi > π && psi < rad2deg(HEADING_UPPER_TURN)
        _switch(fpp, LOW_RIGHT)
    elseif state == LOW_RIGHT && phi < -phi_1 + fpp.fpca._azimuth_offset_phi1
        fpp.fpca.fig8 += 1
        # print "LOW_TURN; phi, psi:", form(phi), form(degrees(psi))
        _switch(fpp, LOW_TURN)
    elseif state == LOW_TURN && psi < rad2deg(180.0 + HEADING_OFFSET_INT) # && phi > -phi_1 - DELTA_PHI_1:
        # print "LOW_TURN_ENDS; phi, beta:", form(phi), form(degrees(psi))            
        _switch(fpp, LOW_LEFT)
    elseif state == LOW_LEFT  && phi > -phi_2 + AZIMUTH_OFFSET_PHI_2 # &&
                                 # beta < (fpp.fpca._beta_set + fpp.fpca._radius + 0.5 * fpp.fpca.elevation_offset +
                                 #         fpp.fpca._elevation_offset_t2)
        _switch(fpp, TURN_LEFT)
    # see: Table 5.5
    elseif state == FLY_LEFT  && phi > fpp.fpca._phi_sw &&
           beta > (fpp.fpca._beta_set + fpp.fpca._radius + 0.5 * fpp.fpca.elevation_offset - DELTA_BETA) &&
           beta < (fpp.fpca._beta_set + fpp.fpca._radius + 0.5 * fpp.fpca.elevation_offset + 2.3)
        fpp.fpca.fig8 += 1            
        _switch(fpp, TURN_LEFT)
    elseif state == TURN_LEFT && psi > deg2rad(180.0 - fpp.fpca._heading_offset)# && phi < fpp._phi_sw + dphi
        _switch(fpp, FLY_RIGHT)
    elseif state == FLY_RIGHT && phi >= phi_3
        if ! fpp.finish
            fpp.finish = (length > fpp.l_up || height > fpp.z_up)
        end
    elseif state == FLY_RIGHT && fpp.finish && phi < phi_3
        _switch(fpp, UP_TURN_LEFT)
    elseif state == FLY_RIGHT && phi < -fpp.fpca._phi_sw # && 
           # beta > (fpp.fpca._beta_set + fpp.fpca._radius + 0.5 * fpp.fpca.elevation_offset - DELTA_BETA)
        _switch(fpp, TURN_RIGHT)
    elseif state == TURN_RIGHT && psi < deg2rad(180.0 + fpp.fpca._heading_offset) # && phi > -fpp._phi_sw - dphi
        _switch(fpp, FLY_LEFT)
    # check, if the condition for finishing is fullfilled while still beeing on the left h&& side
    #    of the wind window
    elseif state == FLY_LEFT && phi <= -phi_3
        if ! fpp.finish
            fpp.finish = (length > fpp.l_up || height > fpp.z_up)
        end
    elseif state == FLY_LEFT && fpp.finish && phi > -phi_3
        _switch(fpp, UP_TURN)
    elseif state == UP_TURN && (psi > deg2rad(360.0 - HEADING_OFFSET_UP) || psi < deg2rad(HEADING_OFFSET_UP))
        _switch(fpp, UP_FLY_UP)
    elseif state == UP_TURN_LEFT && (psi > deg2rad(360.0 - HEADING_OFFSET_UP) || psi < deg2rad(HEADING_OFFSET_UP))
        _switch(fpp, UP_FLY_UP)
    elseif state == UP_FLY_UP && ((fpp.fpca._beta > fpp.fpca._beta_ri) || (height > fpp.z_up) || length > (fpp.l_up + 57.0))
        _switch(fpp, DEPOWER)
    # see: Table 5.3
    elseif state == DEPOWER && length < fpp.l_low
        fpp.fpca.fig8 = 0
        _switch(fpp, POWER)
    end
    # print some debug info every second
    fpp.count += 1
    if fpp.count >= 50
        u_s = fpp.fpca.fpc.u_s
        chi_set = fpp.fpca.fpc.chi_set
        chi_factor = fpp.fpca.fpc.chi_factor
        if PRINT_EVERY_SECOND
            # print "omega, phi, beta, state, chi_factor", form(fpp._omega), form(fpp._phi), form(fpp._beta),\
            #                                                          fpp._state, form(chi_factor)
            # print "--> heading, course, bearing, steering", form(degrees(fpp.fpca.fpc.psi)), \
            #                     form(degrees(fpp.fpca.fpc.chi)), form(rad2deg(chi_set)), form(100 * u_s)
            # turning, value = fpp.fpca.fpc.get_state()
            # if turning
            #     print "--> turning. psi_dot_set: ", form(degrees(value))
            # else
            #     print "--> no turn. attractor:   ", form(degrees(value[begin])), form(degrees(value[begin+1]))
            #     print "beta_set, intermediate", form(fpp._beta_set), fpp.fpc.intermediate
            # end
        end
        fpp.count = 0
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
    if PRINT
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

#     def _calcMidPoint(self, point1, point2)
#         """
#         Calculate the mid point of two points on a sphere.
#         """
#         dazi = point2[begin] - point1[begin]
#         Bx = cos(point2[begin+1]) * cos(dazi)
#         By = cos(point2[begin+1]) * sin(dazi)
#         midpoint = np.zeros(2)
#         midpoint[begin] = point1[begin] + atan2(By, cos(point1[begin+1]) + Bx)
#         midpoint[begin+1] = atan2(sin(point1[begin+1]) + sin(point2[begin+1]), sqrt((cos(point1[begin+1]) + Bx)^2) + By^2)
#         return midpoint


#     """
#     message WayPoint {
#        required double azimuth   = 1; // Angle in radians. Zero straight downwind. Positive direction clockwise seen
#                                       // from above. Valid range -pi .. pi.
#                                       // Upwind is the direction the wind is coming from.
#        required double elevation = 2; // Angle in radians above the horizon. Valid range -pi/2 to pi/2.
#     }

#     // track from the current kite position to nearest waypoint of the desired high level trajectory
#     message Bearing {
#         required int32 counter              = 1; // sequence number of the package; limited to 16 bit
#         repeated WayPoint bearing           = 2; // list of waypoints, recalculated on every FAST_CLOCK event,
#                                                  // when an autopilot is active
#         required double time_sent           = 3; // time, when the bearing was sent from the controller
#     }
#     """
#     def calcSteering(self, parking, period_time)
#         """
#         Calculate the steering and send a bearing vector to be drawn in frontview.
#         Bearing vector Desired trajectory to the next attractor point.
#         """
#         result = self.fpc.calcSteering(parking, period_time)
#         p1, p3 = np.zeros(2), np.zeros(2)
#         p1[begin], p1[begin+1] = -self.fpc.phi, self.fpc.beta
#         p3 = self.fpc.attractor * np.array((-1.0, 1.0))
#         p2 = self._calcMidPoint(p1, p3)
#         self.pub.publishBearing(p1, p2, p3)
#         return result

#  Switch the state of the FPP. Execute all actions, that are needed when the new state is entered.
#  Return immidiately, if the new state is equal to the old state.
function _switch(fpp::FlightPathPlanner, state, delta_beta = 0.0)
    global depower
    if state == fpp._state
        return
    end
    psi_dot_turn = fpp.fpca._omega / fpp.fpca._radius # desired turn rate during the turns
    sys_state = fpp.fpca._sys_state
    # see Table 5.3
    if state == POWER
        depower[] = fpp.u_d_ro + fpp.delta_depower
        sys_state = ssPower
    elseif state == UPPER_TURN
        _publish_fpc_command(fpp, true, psi_dot = PSI_DOT_MAX, attractor=fpp.fpca._p1, intermediate=true)
        sys_state = ssIntermediate
        # see Table 5.4
    elseif state == LOW_RIGHT
        println("LOW_RIGHT, p1: ", fpp.fpca._p1)
        _publish_fpc_command(fpp, false, attractor = fpp.fpca._p1, intermediate = true)
        sys_state = ssIntermediate
    elseif state == LOW_TURN
        p2 = addy(fpp.fpca._p2, fpp.fpca._elevation_offset_p2)
        println("LOW_TURN, p2: ", p2, ", fpp.fpca._p2: ", fpp.fpca._p2)
        _publish_fpc_command(fpp, true, psi_dot = psi_dot_turn, radius=fpp.fpca._radius, attractor = p2, intermediate = true)
        sys_state = ssIntermediate
    elseif state == LOW_LEFT
        p2 = addy(fpp.fpca._p2, fpp.fpca._elevation_offset_p2)
        println("LOW_LEFT, p2: ", p2, ", fpp.fpca._p2: ", fpp.fpca._p2)
        _publish_fpc_command(fpp, false, attractor = p2, intermediate = true)
        sys_state = ssIntermediate
    # see Table 5.5
    elseif state == TURN_LEFT
        radius = -fpp.fpca._radius
        if CORRECT_RADIUS && fpp.fpca.fig8 == 0
            radius *= 0.8
        end
        if PRINT
#                 println("======>>> self.fig8, radius", self.fig8, form(radius))
        end
        _publish_fpc_command(fpp, true, psi_dot = -psi_dot_turn, radius=radius) #,  attractor = fpp.fpca._p3)
        sys_state = ssKiteReelOut
    elseif state == FLY_RIGHT
        if fpp.fpca.fig8 == 0
            if fpp.fpca.high
                _publish_fpc_command(fpp, false, attractor = fpp.fpca._p3_zero_high)
                # println("AAA")
            else
                _publish_fpc_command(fpp, false, attractor = fpp.fpca._p3_zero)
                # println("BBB")
            end
        elseif fpp.fpca.fig8 == 1 && fpp.fpca.high
            _publish_fpc_command(fpp, false, attractor = fpp.fpca._p3_one_high)
            # println("CCC")
        else
            _publish_fpc_command(fpp, false, attractor = fpp.fpca._p3)
            # println("DDD")             
        end
        sys_state = ssKiteReelOut
    elseif state == TURN_RIGHT
        radius = fpp.fpca._radius
        if CORRECT_RADIUS
            radius += (fpp.fpca._beta - (fpp.fpca._t2[begin+1])) * 0.5
            if radius < 1.5
                radius = 1.5
            end
        end
        _publish_fpc_command(fpp, true, psi_dot = psi_dot_turn, radius=radius, attractor = fpp.fpca._p4)
        sys_state = ssKiteReelOut        
    elseif state == FLY_LEFT
        if fpp.fpca.fig8 == 0 && ! fpp.fpca.high               
            _publish_fpc_command(fpp, false, attractor = fpp.fpca._p4_zero)
        elseif fpp.fpca.fig8 == 1
            if ! fpp.fpca.high
                _publish_fpc_command(fpp, false, attractor = fpp.fpca._p4_one)
            else                    
                _publish_fpc_command(fpp, false, attractor = fpp.fpca._p4_one_high)
            end
        else
            if delta_beta < 10.0
                delta_beta = 0.0
            else
                delta_beta -= 10.0
            end
            y = -1.5 * delta_beta
            x = delta_beta
            if delta_beta > 0.0
#              print "--->>> x, y=", form(x), form(y)
            end
            _publish_fpc_command(fpp, false, attractor = addxy(fpp.fpca._p4, x, y))
        end
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
        depower[] = fpp.u_d_ri # Table 5.3
        sys_state = ssDepower
    elseif state == PARKING
        _publish_fpc_command(fpp, false, attractor = fpp.fpca._zenith)
        depower[] = fpp.u_d_pa # Table 5.3
        sys_state = ssParking
    end

    if sys_state != fpp.fpca._sys_state
        if PRINT
            println("############## -->> ", sys_state)
        end
        fpp.fpca._sys_state = sys_state
        on_new_system_state(fpp.fpca, sys_state, true)
        sleep(0.001)
    end
    if PRINT
        println("Switching to: ", state)
    end
    fpp._state = state
end

#     def getFPP_State(self)
#         """ Return the state of the flight path planner (instance of the enum FPPS). """
#         return self._state

#     Highest level state machine
#     Minimal set of states
#     ssParking, ssPowerProduction, ssReelIn
@with_kw mutable struct SystemStateControl @deftype Float64
    wc::WinchController
    # fpc::FlightPathController
    fpp::FlightPathPlanner
    sys_state::Union{Nothing, SysState}    = nothing
    state::Observable(SystemState)[]       = ssParking
    tether_length::Union{Nothing, Float64} = nothing
end

function SystemStateControl(wcs::WCSettings, fcs::FPCSettings, fpps::FPPSettings)
    # fcs.gain = 0.01
    fpc = FlightPathController(fcs)
    fpca = FlightPathCalculator(fpc)
    fpp = FlightPathPlanner(fpps, fpca)

    res = SystemStateControl(wc=WinchController(wcs), fpp=fpp)
    attractor = zeros(2)
    # attractor[begin+1] = deg2rad(25.0) # phi_set
    attractor[2] = deg2rad(80.0) # beta_set
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
    force = ssc.sys_state.force
    v_act = ssc.sys_state.v_reelout
    if ssc.state == ssParking
        v_set = calc_v_set(ssc.wc, v_act, force, f_low, 0.0)
    else
        v_set = calc_v_set(ssc.wc, v_act, force, f_low)
    end
    on_timer(ssc.wc)
    # println(v_set)
    v_set
end

function calc_steering(ssc::SystemStateControl)
    if isnothing(ssc.sys_state)
        return 0.0
    end
    phi  = ssc.sys_state.azimuth
    beta = ssc.sys_state.elevation
    psi = ssc.sys_state.heading
    v_a = ssc.sys_state.v_app
    chi = ssc.sys_state.course
    u_d = ssc.sys_state.depower
    length = ssc.sys_state.l_tether
    height = ssc.sys_state.Z[end]
    omega = 0.0
    # println("phi: $phi, beta: $beta, psi: $psi, chi: $chi, u_d: $u_d")
    # on_est_sysstate(ssc.fpc, phi, beta, psi, chi, omega, v_a; u_d=u_d)
    set_azimuth_elevation(ssc.fpp.fpca, phi, beta)
    on_new_systate(ssc.fpp, phi, beta, psi, chi, v_a, u_d)
    if ssc.state == ssPowerProduction
        on_new_data(ssc.fpp, u_d, length, psi, height)
    end
    u_s = calc_steering(ssc.fpp.fpca.fpc, ssc.state == ssParking)
    on_timer(ssc.fpp.fpca.fpc)
    # println(u_s)
    u_s
end

function switch(ssc::SystemStateControl, state)
    if ssc.state == state && state != ssParking
        return
    end
    ssc.state = state
    # publish new system state
    if state == ssPowerProduction
        start(ssc.fpp)
    end
    if state == ssParking
        on_new_system_state(ssc.fpp.fpca, state, true)
        _switch(ssc.fpp, PARKING)
    end
    println("New system state: ", Symbol(ssc.state))
end

function get_depower(ssc::SystemStateControl)
    return depower[]
end
