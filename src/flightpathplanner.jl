# Provides the components FlightPathPlanner, FlightPathCalculator and SystemStateControl. The main class for
# external use is FlightPathPlanner. Implementation as specified in chapter five of
# the PhD thesis of Uwe Fechner.

BETA_SET                 = 24.0 # for test 502
FIXED_ELEVATION          = false
CORRECT_RADIUS           = false
W_FIG                    = 28.0 # valid values: 36, 28

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
    high = false
    elevation_offset = ELEVATION_OFFSET
end

# Event handler for events, received from the GUI. The flight path planner
# must be in sync with the central system state.
function on_new_system_state(fpca::FlightPathCalculator, new_state, internal = false)
    if fpca._sys_state.value == new_state
        return
    end
    fpca._sys_state = SystemState(new_state)
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
    beta_c1, beta_cw
end

# TODO: Implement _calc_k2_k3
#     def _calcK2K3(self, beta_set):
#         if beta_set > 40.0:
#             k2 = beta_set - 40.0
#         else:
#             k2 = 0.0
#         if self._r_min < 4.0:
#             k3 = 4.0 - self._r_min
#         else:
#             k3 = 0.0
#         return k2, k3

# TODO: Implement _calc_t1
#     def _calcT1(self, beta_set):
#         """
#         Calculate azimuth and elevation of the point T1, where the first turn starts.
#         """
#         k2, k3 = self._calcK2K3(beta_set)
#         beta_c1, k, beta_cw = self._calcBetaC1(beta_set)
#         phi_c1 = (beta_c1 - beta_cw - 10.0 * self._k1 * (0.1 * self._radius)**1.2) / k #- k2 * k3 * self._k4
#         self._t1[0] = (self._beta_int * k + phi_c1 - beta_c1 * k) / (k*k + 1.0)
#         self._t1[1] = self._beta_int - self._t1[0] * k

# TODO: Implement calc_p1
#     def calcP1(self, beta_set):
#         """
#         Calculate azimuth and elevation of the intermediate attractor point P1.
#         """
#         self._calcT1(beta_set)
#         self._delta_phi = sqrt(self._delta_min**2 / (1.0 + self._k**2))
#         self._delta_beta = self._k * sqrt(self._delta_min**2 / (1.0 + self._k**2))
#         self._p1[0] = self._t1[0] + self._delta_phi
#         self._p1[1] = self._t1[1] - self._delta_beta + ELEVATION_OFFSET_P1

# TODO: Implement calc_p2
#     def calcP2(self, beta_set):
#         """
#         Calculate azimuth and elevation of the second attractor point P2.
#         """
#         # Calculate azimuth and elevation of the point T2, where the figure-of-eight starts.
#         phi_sw, r = self._phi_sw, self._radius
#         self._t2[0] = -(self._phi_c3 - phi_sw) - self._phi_c3
#         self._t2[1] = sqrt(r * r - (phi_sw - self._phi_c3)**2) + beta_set
#         self._p2[0] = self._t2[0] - self._delta_phi
#         self._p2[1] = (self._t2[1] - self._delta_beta)

# TODO: Implement calc_p3
#     def calcP3(self):
#         """
#         Calculate azimuth and elevation of the third attractor point P3.
#         """
#         # Calculate azimuth and elevation of the point T3, where the right turn starts.
#         self._t3[0] = self._phi_sw
#         self._t3[1] = self._t2[1]
#         self._p3[0] = self._t3[0] + self._delta_phi
#         self._p3_zero[0] = self._p3[0]
#         self._p3_zero_high[0] = self._p3[0]
#         self._p3_one_high[0] = self._p3[0]
#         self._p3[1] = self._t3[1] + self._delta_beta
#         self._p3_zero[1] = self._p3[1] + ELEVATION_OFFSET_P3_ZERO
#         self._p3_zero_high[1] = self._p3[1] + ELEVATION_OFFSET_P3_ZERO_HIGH
#         self._p3_one_high[1] = self._p3[1] + ELEVATION_OFFSET_P3_ONE_HIGH

# TODO: Implement calc_p4
#     def calcP4(self):
#         """
#         Calculate azimuth and elevation of the forth attractor point P4.
#         """
#         # Calculate azimuth and elevation of the point T4, where the left turn starts.
#         self._t4[0] = - self._phi_sw
#         self._t4[1] = self._t3[1]
#         self._p4[0] = self._t4[0] - self._delta_phi
#         self._p4_one[0] = self._p4[0]
#         self._p4_one_high[0] = self._p4[0]
#         self._p4_zero[0] = self._p4[0]
#         self._p4[1] = (self._t4[1] + self._delta_beta) + self._elevation_offset_p4
#         self._p4_one[1] = self._p4[1] + ELEVATION_OFFSET_P4_ONE
#         self._p4_one_high[1] = self._p4[1] + ELEVATION_OFFSET_P4_ONE_HIGH
#         self._p4_zero[1] = self._p4[1] + ELEVATION_OFFSET_P4_ZERO

# TODO: Implement calc_t5
#     def calcT5(self, beta_set):
#         """
#         Calculate azimuth and elevation of the point T5, where the up-turn starts.
#         """
#         r, k = self._radius, self._k
#         self._t5[0] = r - sqrt(k * k * r * r / (k * k + 1.0))
#         self._t5[1] = beta_set - k * self._t5[0]

# TODO: Implement publish
#     def publish(self, beta_set = BETA_SET):
#         """ Calculate and publish the planned flight path. Must be called each time when
#         the winch controller calculates a new set value for the elevation, but also, when beta_int
#         changes (at the beginning of each intermediate phase). """
#         if FIXED_ELEVATION:
#             beta_set = BETA_SET
#         if beta_set > 30.0:
#             self._w_fig = W_FIG + 10.0
#         if beta_set > 52.0:
#             self._w_fig = W_FIG +  5.0            
#         else:
#             self._w_fig = W_FIG            
#         if beta_set >= 47.5:
#             self._heading_offset = HEADING_OFFSET_HIGH
#             self._dphi = DPHI_HIGH
#             self._elevation_offset_p4 = ELEVATION_OFFSET_P4_HIGH
#         else:
#             if self._v_wind_gnd > 9.2:
#                 self._heading_offset = HEADING_OFFSET_LOW + 4.0
#             else:
#                 self._heading_offset = HEADING_OFFSET_LOW
#             self._dphi = DPHI_LOW
#             self._elevation_offset_p4 = 0.0
#         rel_elevation = (beta_set - 20.0) / 20.0
#         self._beta_set = beta_set
#         self.elevation_offset = ELEVATION_OFFSET + rel_elevation * ELEVATION_OFFSET_40
#         beta_set += self.elevation_offset

#         self.calcP1(beta_set)
#         self.calcP2(beta_set)
#         self.calcP3()
#         self.calcP4()
#         self.calcT5(beta_set)
#         if TRANS_FACTOR >= 1.5:
#             self._t1[0] /= TRANS_FACTOR
#         self._p1[0] /= TRANS_FACTOR
#         phi_1 = self._t1[0]
#         if PRINT:
#             print "phi_1, _p1[0], beta_set, beta_int", form(phi_1), form(self._p1[0]), form(beta_set), \
#                                                        form(self._beta_int)
#         self._phi_2 = self._t2[0]
#         self._phi_3 = self._t5[0]
#         self._beta_ri = self._k5 + self._k6 * beta_set + ELEVATION_OFFSET
#         if self.pub is not None:
#             self.pub.publishPlannedFlightPath(self._p1, self._p2, self._p3, self._p4, self._t1[0], self._t2[0], \
#                                               self._t5[0], self._phi_sw, self._beta_ri)

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



# class FlightPathPlanner(FlightPathCalculator):
#     """
#     Class, that implements the state machine as described in the PhD thesis of Uwe Fechner, chapter
#     five. It inherits from the flight path calculator. It uses the pre-calculated flight path together
#     with incoming kite state data to calculate the FPC_Command messages, if needed.

#     Inputs:
#     a) the inherited flight path and angular kite speed omega (on the unit circle)
#     b) the actual depower value of the kite
#     c) the actual reel-out length of the tether
#     d) the actual orientation of the kite: the heading angle
#     e) the height of the kite

#     Output:
#     FPC_Command messages as defined above.
#     """
#     def __init__(self, pro, clear=false, publisher=None):
#         if clear:
#             super(FlightPathPlanner, self).clear(pro, publisher)
#         else:
#             super(FlightPathPlanner, self).__init__(pro)
#         self._state = FPPS.INITIAL
#         self.delta_depower = 0.0 # this value must be increased, if the power is too high
#         self.const_dd      = 0.7 # greek delta_depower
#         self.u_d_ro =  0.01 * pro._mode.min_depower  # min_depower
#         self.u_d_ri =  0.01 * pro._mode.max_depower # max_depower
#         self.u_d_pa = 0.25   # parking depower
#         self.l_low = pro._mode.min_length   # lower length
#         self.l_up  = pro._mode.max_length   # upper lenght
#         self.z_up  = pro._mode.max_height   # upper height
#         self.count = 0
#         self.finish = false

#     def clear(self, pro, publisher=None):
#         self.__init__(pro, clear=true, publisher=publisher)

#     def setLlowLup(self, l_low, l_up):
#         self.l_low = l_low
#         self.l_up = l_up

#     def start(self):
#         """
#         Start automated power production; Precondition: The kite is parking at a high elevation angle.
#         """
#         if self._sys_state == SystemState.ssManualOperation or self._sys_state == SystemState.ssParking:
#             # see: Table 5.3
#             self._switch(FPPS.POWER)

#     def isActive(self):
#         """ Check, if the new flight path planner is active. """
#         return self._state != FPPS.INITIAL

#     def getState(self):
#         """ Return the state of the flight path planner as integer for logging. """
#         return self._state.value

#     def onNewEstSysState(self, period_time, phi, beta, heading, course, v_a, u_d):
#         """
#         Parameters:
#         phi:  the azimuth angle of the kite position in radian
#         beta: the elevation angle of the kite position in radian
#         psi:  heading of the kite in radian
#         u_d:  relative depower of the kite (0..1)
#         """
#         psi = wrapToPi(heading)
#         chi = wrapToPi(course)
#         self.fpc.onNewEstSysState(-phi, beta, -psi, -chi, self._omega, v_a, u_d=u_d, period_time=period_time)

#     def onNewData(self, depower, length, heading, height, time):
#         """
#         Parameters:
#         depower: 0.0 .. 1.0
#         length: tether length [m]
#         heading: 0 .. 2 pi (psi)
#         height: height [m]

#         Inherited:
#         self._phi:   azimuth in degrees
#         self._beta:  elevation in degrees
#         self._omega: angular speed in degrees per second

#         Determine, if a state change is need it and change it by calling the switch method, if neccessary.
#         """
#         phi, psi  = self._phi, heading
#         beta = self._beta
#         phi_1 = self._t1[0]
#         phi_2 = self._phi_2
#         phi_3 = self._phi_3
#         state = self._state
#         if self.fig8 == 0:
#             dphi = self._dphi + 5.0 
#         elif self.fig8 <= 2:
#             dphi = self._dphi + 3.0
#         else:
#             dphi = self._dphi + 2.0
#         #if PRINT:
#         #    print "dphi: ", form(dphi)
#         # see: Table 5.3, 5.4
#         if state == FPPS.POWER:
#             self.finish = false
#             # use the intermediate point only if we have to fly down more than 20 degrees
#             delta_beta = beta - self._beta_set - self._radius
#             if (beta > self._beta_set + 25.0 + self._radius) and not DIRECT:
#                 if depower < self.u_d_ro + self.delta_depower + self.const_dd * \
#                                                            (self.u_d_ri - self.u_d_ro - self.delta_depower):
#                     if PRINT_DELTA_BETA:
#                         print "Delta beta large: ", form(delta_beta)  
#                     self.fig8 = -1
#                     self.high = false
#                     self._switch(FPPS.UPPER_TURN)
#             else:
#                 if depower < self.u_d_ro + self.delta_depower + self.const_dd * \
#                                                            (self.u_d_ri - self.u_d_ro - self.delta_depower):
#                     if PRINT_DELTA_BETA:
#                         print "Delta beta low: ", form(delta_beta)                                                          
#                     self.fig8 = -1
#                     self.high = true
#                     self._switch(FPPS.FLY_LEFT, delta_beta)
#         elif state == FPPS.UPPER_TURN and psi > pi and psi < radians(HEADING_UPPER_TURN):
#             self._switch(FPPS.LOW_RIGHT)
#         elif state == FPPS.LOW_RIGHT and phi < -phi_1 + self._azimuth_offset_phi1:
#             self.fig8 += 1
#             print "LOW_TURN; phi, psi:", form(phi), form(degrees(psi))
#             self._switch(FPPS.LOW_TURN)
#         elif state == FPPS.LOW_TURN  and psi < radians(180.0 + HEADING_OFFSET_INT): # and phi > -phi_1 - DELTA_PHI_1:
#             if PRINT:
#                 print "===>>> time, phi, _phi_sw + dphi, psi", form(time), form(phi), form(-phi_1 - DELTA_PHI_1), \
#                                                             form(degrees(psi))
#             print "LOW_TURN_ENDS; phi, beta:", form(phi), form(degrees(psi))            
#             self._switch(FPPS.LOW_LEFT)
#         elif state == FPPS.LOW_LEFT  and phi > -phi_2 + AZIMUTH_OFFSET_PHI_2 \
#                                      and beta < (self._beta_set + self._radius + 0.5 * self.elevation_offset + \
#                                          self._elevation_offset_t2):
#             self._switch(FPPS.TURN_LEFT)
#         # see: Table 5.5
#         elif state == FPPS.FLY_LEFT  and phi > self._phi_sw \
#                                      and beta > (self._beta_set + self._radius + 0.5 * self.elevation_offset - DELTA_BETA) \
#                                      and beta < (self._beta_set + self._radius + 0.5 * self.elevation_offset + 2.3):
#             self.fig8 += 1            
#             self._switch(FPPS.TURN_LEFT)
#         elif state == FPPS.TURN_LEFT and psi > radians(180.0 - self._heading_offset):# and phi < self._phi_sw + dphi:
#             if PRINT:
#                 print "===>>> time, phi, _phi_sw + dphi, psi, fig8", form(time), form(phi), \
#                                               form(self._phi_sw + dphi), form(degrees(psi)), form(self.fig8)
#             self._switch(FPPS.FLY_RIGHT)
#         elif state == FPPS.FLY_RIGHT and phi >= phi_3:
#             if not self.finish:
#                 self.finish = (length > self.l_up or height > self.z_up)
#                 if self.finish and PRINT:
#                     print '=========> Set finish to true! '
#         elif state == FPPS.FLY_RIGHT and self.finish and phi < phi_3:
#             self._switch(FPPS.UP_TURN_LEFT)
#         elif state == FPPS.FLY_RIGHT and phi < -self._phi_sw \
#                                      and beta > (self._beta_set + self._radius + 0.5 * self.elevation_offset - DELTA_BETA):
#             self._switch(FPPS.TURN_RIGHT)
#         elif state == FPPS.TURN_RIGHT and psi < radians(180.0 + self._heading_offset): # and phi > -self._phi_sw - dphi:
#             self._switch(FPPS.FLY_LEFT)
#         # check, if the condition for finishing is fullfilled while still beeing on the left hand side
#         #    of the wind window
#         elif state == FPPS.FLY_LEFT and phi <= -phi_3:
#             if not self.finish:
#                 self.finish = (length > self.l_up or height > self.z_up)
#                 if self.finish and PRINT:
#                     print '=========> Set finish to true! '
#         elif state == FPPS.FLY_LEFT and self.finish and phi > -phi_3:
#             self._switch(FPPS.UP_TURN)
#         elif state == FPPS.UP_TURN and (psi > radians(360.0 - HEADING_OFFSET_UP) or psi < radians(HEADING_OFFSET_UP)):
#             self._switch(FPPS.UP_FLY_UP)
#         elif state == FPPS.UP_TURN_LEFT and (psi > radians(360.0 - HEADING_OFFSET_UP) or psi < radians(HEADING_OFFSET_UP)):
#             self._switch(FPPS.UP_FLY_UP)
#         elif state == FPPS.UP_FLY_UP and ((self._beta > self._beta_ri) or (height > self.z_up) or length > (self.l_up + 57.0)):
#             self._switch(FPPS.DEPOWER)
#         # see: Table 5.3
#         elif state == FPPS.DEPOWER and length < self.l_low:
#             self.fig8 = 0
#             self._switch(FPPS.POWER)
#         # print some debug info every second
#         self.count += 1
#         if self.count >= 50:
#             u_s = self.fpc.getSteering()
#             chi_set = self.fpc.chi_set
#             chi_factor = self.fpc.getChiFactor()
#             if PRINT_EVERY_SECOND:
#                 print "omega, phi, beta, state, chi_factor", form(self._omega), form(self._phi), form(self._beta),\
#                                                          self._state, form(chi_factor)
#                 print "--> heading, course, bearing, steering", form(degrees(self.fpc.psi)), \
#                              form(degrees(self.fpc.chi)), form(degrees(chi_set)), form(100 * u_s)
#                 turning, value = self.fpc.getState()
#                 if turning:
#                     print "--> turning. psi_dot_set: ", form(degrees(value))
#                 else:
#                     print "--> no turn. attractor:   ", form(degrees(value[0])), form(degrees(value[1]))
#                 print "beta_set, intermediate", form(self._beta_set), self.fpc.intermediate
#             self.count = 0

#     def _publishFPC_Command(self, turn, attractor=None, psi_dot=None, radius=None, intermediate = false):
#         """
#         Publish a new command of the flight path planner and call the related method of the
#         flight path controller directly. Publishing is currently only relevant for logging and debugging.
#         """
#         if psi_dot is not None:
#             psi_dot = degrees(psi_dot)
#         fpc_attactor = attractor * np.array((-1.0, 1.0))
#         self.pub2.publishFPC_Command(turn, fpc_attactor, psi_dot)
#         self.fpc.onNewControlCommand(attractor=np.radians(fpc_attactor), psi_dot_set=psi_dot, radius=radius, intermediate = intermediate)
#         if PRINT:
#             print "New FPC command. Intermediate: ", intermediate
#             if psi_dot is None:
#                 print "New attractor point:", form(fpc_attactor[0]), form(fpc_attactor[1])
#             else:
#                 if radius is None:
#                     print "New psi_dot_set [°/s] ", form(psi_dot)
#                 else:
#                     print "New psi_dot_set [°/s], radius [°] ", form(psi_dot), form(radius)

#     def _calcMidPoint(self, point1, point2):
#         """
#         Calculate the mid point of two points on a sphere.
#         """
#         dazi = point2[0] - point1[0]
#         Bx = cos(point2[1]) * cos(dazi)
#         By = cos(point2[1]) * sin(dazi)
#         midpoint = np.zeros(2)
#         midpoint[0] = point1[0] + atan2(By, cos(point1[1]) + Bx)
#         midpoint[1] = atan2(sin(point1[1]) + sin(point2[1]), sqrt((cos(point1[1]) + Bx)**2) + By**2)
#         return midpoint


#     """
#     message WayPoint {
#        required double azimuth   = 1; // Angle in radians. Zero straight downwind. Positive direction clockwise seen
#                                       // from above. Valid range: -pi .. pi.
#                                       // Upwind is the direction the wind is coming from.
#        required double elevation = 2; // Angle in radians above the horizon. Valid range: -pi/2 to pi/2.
#     }

#     // track from the current kite position to nearest waypoint of the desired high level trajectory
#     message Bearing {
#         required int32 counter              = 1; // sequence number of the package; limited to 16 bit
#         repeated WayPoint bearing           = 2; // list of waypoints, recalculated on every FAST_CLOCK event,
#                                                  // when an autopilot is active
#         required double time_sent           = 3; // time, when the bearing was sent from the controller
#     }
#     """
#     def calcSteering(self, parking, period_time):
#         """
#         Calculate the steering and send a bearing vector to be drawn in frontview.
#         Bearing vector: Desired trajectory to the next attractor point.
#         """
#         result = self.fpc.calcSteering(parking, period_time)
#         p1, p3 = np.zeros(2), np.zeros(2)
#         p1[0], p1[1] = -self.fpc.phi, self.fpc.beta
#         p3 = self.fpc.attractor * np.array((-1.0, 1.0))
#         p2 = self._calcMidPoint(p1, p3)
#         self.pub.publishBearing(p1, p2, p3)
#         return result

#     def _switch(self, state, delta_beta = 0.0):
#         """
#         Switch the state of the FPP. Execute all actions, that are needed when the new state is entered.
#         Return immidiately, if the new state is equal to the old state.
#         """
#         if state == self._state:
#             return
#         psi_dot_turn = self._omega / self._radius # desired turn rate during the turns
#         # see: Table 5.3
#         if state == FPPS.POWER:
#             depower = self.u_d_ro + self.delta_depower
#             self.pub2.sendEvent(system.KITE_CTRL_CMD, depower = depower * 100.0) # Table 5.3
#             sys_state = SystemState.ssPower
#         if state == FPPS.UPPER_TURN:
#             self._publishFPC_Command(true, psi_dot = PSI_DOT_MAX, attractor = self._p1, \
#             intermediate = true)
#             sys_state = SystemState.ssIntermediate
#         # see: Table 5.4
#         elif state == FPPS.LOW_RIGHT:
#             self._publishFPC_Command(false, attractor = self._p1, intermediate = true)
#             sys_state = SystemState.ssIntermediate
#         elif state == FPPS.LOW_TURN:
#             p2 = addy(self._p2, self._elevation_offset_p2)
#             self._publishFPC_Command(true, psi_dot = psi_dot_turn, radius=self._radius, attractor = p2, intermediate = true)
#             sys_state = SystemState.ssIntermediate
#         elif state == FPPS.LOW_LEFT:
#             p2 = addy(self._p2, self._elevation_offset_p2)
#             self._publishFPC_Command(false, attractor = p2, intermediate = true)
#             sys_state = SystemState.ssIntermediate
#         # see: Table 5.5
#         elif state == FPPS.TURN_LEFT:
#             radius = -self._radius
#             if CORRECT_RADIUS and self.fig8 == 0:
#                 radius *= 0.8
#             if PRINT:
#                 print "======>>> self.fig8, radius", self.fig8, form(radius)
#             self._publishFPC_Command(true, psi_dot = -psi_dot_turn, radius=radius,  attractor = self._p3)
#             sys_state = SystemState.ssKiteReelOut
#         elif state == FPPS.FLY_RIGHT:
#             if self.fig8 == 0:
#                 if self.high:
#                     self._publishFPC_Command(false, attractor = self._p3_zero_high)
#                     # print "AAA"
#                 else:
#                     self._publishFPC_Command(false, attractor = self._p3_zero)
#                     # print "BBB"
#             elif self.fig8 == 1 and self.high:
#                 self._publishFPC_Command(false, attractor = self._p3_one_high)
#                 # print "CCC"
#             else:
#                 self._publishFPC_Command(false, attractor = self._p3)
#                 # print "DDD"
#             sys_state = SystemState.ssKiteReelOut
#         elif state == FPPS.TURN_RIGHT:
#             radius = self._radius
#             if CORRECT_RADIUS:
#                 radius += (self._beta - (self._t2[1])) * 0.5
#                 if radius < 1.5:
#                     radius = 1.5
#             self._publishFPC_Command(true, psi_dot = psi_dot_turn, radius=radius, attractor = self._p4)
#             sys_state = SystemState.ssKiteReelOut
#         elif state == FPPS.FLY_LEFT:
#             if self.fig8 == 0 and not self.high:
                
#                 self._publishFPC_Command(false, attractor = self._p4_zero)
#             if self.fig8 == 1:
#                 if not self.high:
#                     self._publishFPC_Command(false, attractor = self._p4_one)
#                 else:
                    
#                     self._publishFPC_Command(false, attractor = self._p4_one_high)
#             else:
#                 if delta_beta < 10.0:
#                     delta_beta = 0.0
#                 else:
#                     delta_beta -= 10.0
#                 y = -1.5 * delta_beta
#                 x = delta_beta
#                 if delta_beta > 0.0:
#                     print "--->>> x, y=", form(x), form(y)
#                 self._publishFPC_Command(false, attractor = addxy(self._p4, x, y))
#             sys_state = SystemState.ssKiteReelOut
#         # see: Table 5.6
#         elif state == FPPS.UP_TURN:
#             self._publishFPC_Command(true, psi_dot = psi_dot_turn, radius=self._radius, attractor = self._zenith)
#             sys_state = SystemState.ssWaitUntil
#         elif state == FPPS.UP_TURN_LEFT:
#             self._publishFPC_Command(true, psi_dot = -psi_dot_turn, radius=-self._radius, attractor = self._zenith)
#             sys_state = SystemState.ssWaitUntil
#         elif state == FPPS.UP_FLY_UP:
#             self._publishFPC_Command(false, attractor = self._zenith)
#             sys_state = SystemState.ssWaitUntil
#         elif state == FPPS.DEPOWER:
#             self._publishFPC_Command(false, attractor = self._zenith)
#             self.pub2.sendEvent(system.KITE_CTRL_CMD, depower = self.u_d_ri * 100.0) # Table 5.3
#             sys_state = SystemState.ssDepower
#         elif state == FPPS.PARKING:
#             self._publishFPC_Command(false, attractor = self._zenith)
#             self.pub2.sendEvent(system.KITE_CTRL_CMD, depower = self.u_d_pa * 100.0) # Table 5.3
#             sys_state = SystemState.ssParking

#         if sys_state.value != self._sys_state.value:
#             if PRINT:
#                 print "############## -->>>>>>>", sys_state, self._sys_state
#             self._sys_state = sys_state
#             self.onNewSystemState(sys_state.value, true)
#             self.pub2.sendEvent(system.NEW_SYSTEM_STATE, system_state=sys_state.value)
#             time.sleep(0.001)


#         if PRINT:
#             print "Switching to: ", state
#         self._state = state

#     def getFPP_State(self):
#         """ Return the state of the flight path planner (instance of the enum FPPS). """
#         return self._state

#     Highest level state machine
#     Minimal set of states:
#     ssParking, ssPowerProduction, ssReelIn
@with_kw mutable struct SystemStateControl @deftype Float64
    wc::WinchController
    fpc::FlightPathController
    sys_state::Union{Nothing, SysState}    = nothing
    state::Observable(SystemState)[]       = ssParking
    tether_length::Union{Nothing, Float64} = nothing
end

function SystemStateControl(wcs::WCSettings, fcs::FPCSettings)
    # fcs.gain = 0.01
    res = SystemStateControl(wc=WinchController(wcs), fpc=FlightPathController(fcs))
    attractor = zeros(2)
    # attractor[1] = deg2rad(25.0) # phi_set
    attractor[2] = deg2rad(80.0) # beta_set
    on_control_command(res.fpc, attractor=attractor)
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
    if ssc.state == ssReelIn
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
    omega = 0.0
    # println("phi: $phi, beta: $beta, psi: $psi, chi: $chi, u_d: $u_d")
    on_est_sysstate(ssc.fpc, phi, beta, psi, chi, omega, v_a; u_d=u_d)
    u_s = -calc_steering(ssc.fpc, false)
    on_timer(ssc.fpc)
    println(u_s)
    u_s
end

function switch(ssc::SystemStateControl, state)
    if ssc.state == state && state != ssParking
        return
    end
    ssc.state = state
    # publish new system state
    println("New system state: ", Symbol(ssc.state))
end
