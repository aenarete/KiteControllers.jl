# Provides the components FlightPathCalculator. Implementation as specified in chapter five of
# the PhD thesis of Uwe Fechner.

depower::Float64 = 0.0 # set value of the depower value, 0 .. 1

HEADING_OFFSET_INT =  32.0 # dito, for the turn around the intermediate point
HEADING_OFFSET_HIGH = 54.0 # dito, for elevation angles > 47.5 degrees
HEADING_OFFSET_UP   = 60.0 # degrees, before finishing the up-turn
HEADING_UPPER_TURN =  360.0-25.0

function addy(vec, y)
    SVector(vec[begin], vec[begin+1]+y)
end
  
function addxy(vec, x, y)
    SVector(vec[begin]+x, vec[begin+1]+y)
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
    fpps::FPPSettings
    _beta_min = 20.0 # minimal elevation angle of the center of the figure of eight
    _beta_max = 60.0 # maximal elevation angle of the center of the figure of eight

    _r_min =  3.0 # minimal turn radius in degrees
    _r_max =  4.5
    _radius = 4.5
    _w_fig = fpps.w_fig # width of the figure of eight in degrees
    _phi_c3 = 0.0
    _beta_set = fpps.beta_set # average elevation angle during reel-out
    _beta_int = 68.5 # elevation angle at the start of beginning of the ssIntermediate
    _k = 0.0 # gradient of straight flight path sections, calculated value
    _k1 = 1.28        # for the calculation of T1
    _k5 = 37.5        # for calculation of beta_reel_in
    _k6 = 0.45        # for calculation of beta_reel_in
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
    _p4::MVector{2, Float64} = zeros(2)
    _zenith::MVector{2, Float64} = MVector(0.0, 90) # desired elevation angle at zenith
    _phi_2 = 0.0
    _phi_3 = 0.0
    _phi_sw = 0.0
    _beta_ri = 0.0
    _heading_offset = fpps.heading_offset_low
    _v_wind_gnd = 6.0 # ground wind speed at 6 m height
    fig8 = 0 # number of the current figure of eight
    cycle::Int64 = 0
    _sys_state::SystemState = ssManualOperation
    high::Bool = false
end

function FlightPathCalculator(fpc::FlightPathController, fpps::FPPSettings)
    FlightPathCalculator(fpc=fpc, fpps=fpps)
end

# Event handler for events, received from the GUI. The flight path planner
# must be in sync with the central system state.
function on_new_system_state(fpca::FlightPathCalculator, new_state::SystemState, internal = false)
    if fpca._sys_state == new_state
        return
    end
    fpca._sys_state = new_state
    if Int(new_state) == Int(ssIntermediate)
        fpca._beta_int = fpca._beta
        # calculate and publish the flight path for the next cycle
        publish(fpca, fpca._beta_set)
        fpca.cycle+=1
    elseif new_state == Int(ssParking) && ! internal
        _switch(fpca, PARKING)
    end
end
            
# Set the ground wind speed at 6 m height
function set_v_wind_gnd(fpca::FlightPathCalculator, v_wind_gnd)
    fpca._v_wind_gnd = v_wind_gnd
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

# Calculate azimuth and elevation of the point T1, where the first turn starts.
function _calc_t1(fpca::FlightPathCalculator, beta_set)
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
     fpca._p1[begin+1] = fpca._t1[begin+1] - fpca._delta_beta
     nothing
end

# Calculate azimuth and elevation of the second attractor point P2.
function calc_p2(fpca::FlightPathCalculator, beta_set)
    # Calculate azimuth and elevation of the point T2, where the figure-of-eight starts.
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
    fpca._p3[begin+1] = fpca._t3[begin+1] + fpca._delta_beta
    nothing
end

# Calculate azimuth and elevation of the forth attractor point P4.
function calc_p4(fpca::FlightPathCalculator)
    # Calculate azimuth and elevation of the point T4, where the left turn starts.
    fpca._t4[begin] = - fpca._phi_sw
    fpca._t4[begin+1] = fpca._t3[begin+1]
    fpca._p4[begin] = fpca._t4[begin] - fpca._delta_phi
    fpca._p4[begin+1] = (fpca._t4[begin+1] + fpca._delta_beta)
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
function publish(fpca::FlightPathCalculator, beta_set = fpca.fpps.beta_set)
    fpca._heading_offset = HEADING_OFFSET_HIGH
    fpca._beta_set = beta_set

    calc_p1(fpca, beta_set)
    calc_p2(fpca, beta_set)
    calc_p3(fpca)
    calc_p4(fpca)
    calc_t5(fpca, beta_set)
    fpca._phi_2 = fpca._t2[begin]
    fpca._phi_3 = fpca._t5[begin]
    fpca._beta_ri = fpca._k5 + fpca._k6 * beta_set
    nothing
end
