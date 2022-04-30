# Implements a FlightPathController as specified in chapter six of
# the PhD thesis of Uwe Fechner.

# USE_CHI = True

# RESET_INT1 = True # reset the main integrator to the last estimated turn rate
# RESET_INT2 = False # reset the integrator of the D part at the second time step
# INIT_OPT_TO_ZERO = False # if the root finder should start with zero
# RESET_INT1_TO_ZERO = True

# TAU = 0.95
const TAU_VA = 0.0 # time constant for averaging the apparent wind velocity

"""
Settings of the FlightPathController
"""
@with_kw mutable struct CourseControlSettings @deftype Float64
    prn::Bool             = false
    prn_ndi_gain::Bool    = false
    prn_est_psi_dot::Bool = false
    prn_va::Bool          = false
    use_radius::Bool      = true
    "P gain of the PID controller"
    p  = 20.0
    "I gain of the PID controller"
    i  = 1.2
    "D gain of the PID controller"
    d  = 10.0
    "additional factor for P, I and D"
    gain = 0.04
    c1 =  0.0612998898221 # was: 0.0786
    c2 =  1.22597628388   # was: 2.508
    "correction factor, used by the NDI block; increase k_c1, if the radius is too small; "
    k_c1 = 1.6
    "c2 for the reelout phase was: 7.0"
    k_c2 = 6.0
    "C2 for the reelout phase at high elevation angles was: 14.0"
    k_c2_high = 12.0
    "C2 for the intermediate phase LOW_RIGHT, LOW_TURN, LOW_LEFT"
    k_c2_int  =  0.6
end

const cc = CourseControlSettings()

"""
FlightPathController as specified in chapter six of the PhD thesis of Uwe Fechner.

Main inputs are calls to the functions:
- on_control_command()
- on_est_sys_state()

Main output is the set value of the steering u_s, returned by the method:
- calc_steering()

Once per time step the method
- onTimer
must be called.

See also:
../doc/flight_path_controller_I.png and
../doc/flight_path_controller_II.png and
../doc/flight_path_controller_III.png
"""
@with_kw mutable struct FlightPathController @deftype Float64
    "cycle number"
    count::Int64                               = 0 
    "attractor coordinates, azimuth and elevation in radian"
    attractor::MVector{2, Float64}             = zeros(2)
    "desired turn rate in rad per second or nothing"
    psi_dot_set::Union{Nothing, Float64}       = nothing
    psi_dot_set_final::Union{Nothing, Float64} = nothing
    "azimuth angle of the kite position in radian"
    phi                                        = 0
    "elevation angle of the kite position in radian"
    beta                                       = 0
    "heading of the kite in radian"
    psi                                        = 0
    "course in radian"
    chi                                        = 0
    "0.0 use psi only; 1.0 use chi only"
    chi_factor                                 = 0
    "angular velocity of the kite in degrees/s"
    omega                                      = 0
    "estimated turn rate"
    est_psi_dot                                = 0
    "desired flight direction (bearing)"
    chi_set                                    = 0
    "minimal value of the depower setting, needed for the fully powered kite"
    u_d0                                       = 0.01 * se().depower_offset
    "maximal value of the depower setting, needed for the fully depowered kite"
    u_d_max                                    = 0.01 * 42.2 # TODO: add this to settings.yaml
    "normalized depower settings"
    u_d_prime                                  = 0.2
    "maximal value of the steering settings"
    u_s_max                                    = 0.99
    "maximal value of the turn rate in radians per second"
    psi_dot_max                                = 3.0
    "influece of the depower settings on the steering sensitivity"
    k_ds                                       = se().k_ds # was 2.0
    "actual depower setting (0..1)"
    u_d                                        = 0.01 * se().depower
    k_c2                                       = cc.k_c2
    k_c2_int                                   = cc.k_c2_int
    k_c2_hight                                 = cc.k_c2_high
    c1                                         = cc.c1 * cc.k_c1  # identified value for hydra kite from paper [rad/m]
    c2                                         = cc.c2 * cc.k_c2
    intermediate::Bool                         = true
    "apparent wind speed at the kite"
    va                                         = 0.0
    "average apparent wind speed"
    va_av                                      = 0.0
    "minimal apparent wind speed for full NDI"
    va_min                                     = 8.0
    "quotient of the output and the input of the NDI block"
    ndi_gain                                   = 1.0
    "integrator for the I part of the pid controller"
    int::Integrator                            = Integrator()
    "integrator for the D part of the pid controller"
    int2::Integrator                           = Integrator()
    "P gain of the PID controller"
    p                                          = cc.p
    "I gain of the PID controller"
    i                                          = cc.i
    "D gain of the PID controller"
    d                                          = cc.d
    "additional factor for P, I and D"
    gain                                       = cc.gain
    "anti-windup gain for limited steering signal"
    k_u                                        =  5.0
    "anti-windup gatin for limited turn rate"
    k_psi                                      = 10.0
    "error (input of the PID controller)"
    err                                        =  0
    "residual of the solver"
    res::MVector{2, Float64}                   = zeros(2)
    "steering output of the FPC, calculated by solve()"
    u_s                                        = 0
    "period time"
    dt                                         = 1.0 / se().sample_freq
    k_psi_out                                  = 0
    k_u_out                                    = 0
    k_psi_in                                   = 0
    k_u_in                                     = 0
    int_in                                     = 0
    int2_in                                    = 0
    reset_int1::Bool                           = false
    radius::Union{Nothing, Float64}            = nothing
    _n                                         = 15
    "number of calls of solve"
    _i                                         = 0
end

"""
    function on_control_command(fpc, attractor=nothing, psi_dot_set=nothing, radius=nothing, intermediate = true)

Input:  
Either the attractor point (MVector of azimuth and elevation in radian),
or psi_dot, the set value for the turn rate in degrees per second.
"""
function on_control_command(fpc, attractor=nothing, psi_dot_set=nothing, radius=nothing, intermediate = true)
    fpc.intermediate = intermediate
    if cc.use_radius && ! isnothing(radius)
        psi_dot_set = rad2deg(fpc.omega / radius) # desired turn rate during the turns
    end
    if ! isnothing(psi_dot_set) && ! isnothing(radius)
        temp = fpc.omega / radius
        if cc.prn
            @printf "--->>--->> temp, psi_dot_set %.2f %.2f %.2f %.2f" temp psi_dot_set omega radius
        end
    end
    fpc.radius = radius
    if isnothing(psi_dot_set) && ! isnothing(fpc.psi_dot_set)
        # reset integrator
        fpc.reset_int1 = True
    end
    if ! isnothing(attractor)
        fpc.attractor .= attractor
    end
    if ! isnothing(psi_dot_set)
        fpc.psi_dot_set_final = deg2rad(psi_dot_set)
        fpc.psi_dot_set = fpc.psi_dot_set_final * 2.0
    else
        fpc.psi_dot_set = nothing
    end
end

"""
    function on_est_sys_state(fpc, phi, beta, psi, chi, omega, v_a; u_d=nothing, u_d_prime=nothing)

Parameters:
- phi:      azimuth angle of the kite position in radian
- beta:     elevation angle of the kite position in radian
- psi:      heading of the kite in radian
- chi:      course of the kite in radian
- omega:    angular velocity of the kite on the unit sphere in degrees/s ???
- u_d:      depower settings             [0..1]
- u_d_prime normalized depower settings  [0..1]

Either u_d or u_d_prime must be provided.
"""
function on_est_sysstate(fpc, phi, beta, psi, chi, omega, va; u_d=nothing, u_d_prime=nothing)
    fpc.phi = phi
    fpc.chi = chi
    fpc.omega = omega
    fpc.beta = beta
    if fpc._i > 0
        delta = psi - fpc.psi
        if delta < -pi
            delta += 2π
        elseif delta > π
            delta -= 2π
            fpc.est_psi_dot = delta / fpc.dt
        end
    end
    fpc.psi = psi
    # Eq. 6.4: calculate the normalized depower setting
    if isnothing(u_d_prime)
        fpc.u_d_prime = (u_d - fpc.u_d0) / (fpc.u_d_max - fpc.u_d0)
    else
        fpc.u_d_prime = u_d_prime
    end
    fpc.u_d = u_d
    fpc.va = va
    fpc.va_av = TAU_VA * fpc.va_av + (1.0 - TAU_VA) * va
    # print some debug info every 2.5 seconds
    fpc.count += 1
    if fpc.count >= 50
        if cc.prn_ndi_gain
            @printf "ndi_gain: %.2f" fpc.ndi_gain
        end
        if cc.prn_est_psi_dot
            @printf "est_psi_dot: %.2f" degrees(fpc.est_psi_dot)
        end
        if cc.prn_va
            @printf "va, va_av: %.2f, %.2f" va fpc.va_av
        end
        fpc.count = 0
    end
end

"""
    function navigate(fpc, limit=50.0)

Calculate the desired flight direction chi_set using great circle navigation.
Limit delta_beta to the value of the parameter limit (in degrees).
"""
function navigate(fpc, limit=50.0)
    # navigate only if steering towards the attractor point is active
    if ! isnothing(fpc.psi_dot_set)
        return nothing
    end
    phi_set  = fpc.attractor[1]
    beta_set = fpc.attractor[2]
    r_limit = deg2rad(limit)
    if beta_set - fpc.beta > r_limit
        beta_set = fpc.beta + r_limit
    elseif beta_set - fpc.beta < -r_limit
        beta_set = fpc.beta - r_limit
    end
    y = sin(phi_set - fpc.phi) * cos(beta_set)
    x = cos(fpc.beta) * sin(beta_set) - sin(fpc.beta) * cos(beta_set) * cos(phi_set - fpc.phi)
    fpc.chi_set = atan(-y, x)
end

function linearize(fpc, psi_dot; fix_va=false)
#         """
#         Implement the nonlinear, dynamic inversion block (NDI) according to Eq. 6.4 and Eq. 6.12.
#         psi_dot: desired turn rate in radians per second
#         fix_va: keep v_a fixed for the second term of the turn rate law; was useful in some
#         simulink tests.
#         """
#         # Eq. 6.13: calculate v_a_hat
#         if self.v_a_av >= self.v_a_min:
#             v_a_hat = self.v_a_av
#         else:
#             v_a_hat = self.v_a_min
#         # print "v_a, v_a_hat", self.v_a, v_a_hat
#         # Eq. 6.12: calculate the steering from the desired turn rate
#         if fix_va:
#             va_fix = 20.0
#         else:
#             va_fix = v_a_hat
#         if self.intermediate:
#             k = (va_fix - 22) / 3.8
#             c2 = C2 * (self.k_c2_int + k)
#         else:
#             k = (va_fix - 22) / 3.5 # was: 4
#             if self.beta < radians(30):
#                 c2 = C2 * (self.k_c2 + k)
#             else:
#                 c2 = C2 * (self.k_c2_high + k)
#         u_s = (1.0 + self.K_d_s * self.u_d_prime) / (self.c1 * v_a_hat) \
#               * (psi_dot - c2 / va_fix * sin(self.psi) * cos(self.beta))
#         if abs(psi_dot) < 1e-6:
#             psi_dot = 1e-6
#         self.ndi_gain = saturation(u_s / psi_dot, -20.0, 20.0)
#         if abs(self.ndi_gain) < 1e-6:
#             self.ndi_gain = 1e-6
#         # print "ndi_gain", self.ndi_gain
#         return u_s
end

#     def _calcSat1In_Sat1Out_SatIn_Sat2Out(self, x):
#         """
#         see: ./01_doc/flight_path_controller_II.png
#         x: vector of k_u_in, k_psi_in and int2_in
#         """
#         k_u_in   = x[0]
#         k_psi_in = x[1]

#         # calculate I part
#         int_in = self.I * self.err + self.K_u * k_u_in + self.K_psi * k_psi_in
#         int_out = self.int.calcOutput(int_in)

#         # calculate D part
#         int2_in = self._N * (self.err * self.D - self.int2.getLastOutput()) / (1.0 + self._N * self.period_time)
#         self.int2.calcOutput(int2_in)

#         # calculate P, I, D output
#         sat1_in = (self.P * self.err + int_out + int2_in) * self.gain

#         # calcuate saturated set value of the turn rate psi_dot
#         sat1_out = saturation(sat1_in, -self.psi_dot_max, self.psi_dot_max)
#         # nonlinar inversion
#         sat2_in = self._linearize(sat1_out)
#         # calculate the saturated set value of the steering
#         sat2_out = saturation(sat2_in, -self.u_s_max, self.u_s_max)
#         return sat1_in, sat1_out, sat2_in, sat2_out, int_in

#     def _calcResidual(self, x):
#         """
#         see: ./01_doc/flight_path_controller_II.png
#         x: vector of k_u_in, k_psi_in and int2_in
#         """
#         sat1_in, sat1_out, sat2_in, sat2_out, int_in = self._calcSat1In_Sat1Out_SatIn_Sat2Out(x)
#         k_u_in = (sat2_out - sat2_in) / self.ndi_gain
#         k_psi_in = sat1_out - sat1_in
#         self.res[0] = k_u_in - x[0]
#         self.res[1] = k_psi_in - x[1]
#         return self.res

#     def _solve(self, parking):
#         """
#         Calculate the steering output u_s and the turn rate error err,
#         but also the signals Kpsi_out, Ku_out and int_in.

#         Implements the simulink block diagram, shown in:
#         01_doc/flight_path_controller_I.png

#         If the parameter parking is True, only the heading is controlled, not the course.
#         """
#         self._navigate()
#         # control the heading of the kite
#         chi_factor = 0.0
#         if self.omega > 0.8:
#              chi_factor = (self.omega - 0.8) / 1.2
#         if chi_factor > 0.85:
#             chi_factor = 0.85
#         self.chi_factor = chi_factor
#         if USE_CHI and not parking:
#             control_var = mergeAngles(self.psi, self.chi, chi_factor)
#         else:
#             self.chi_factor = 0.0
#             control_var = self.psi
#         self.err = wrapToPi(self.chi_set - control_var)
#         if RESET_INT1 and self._i == 0 or self.reset_int1:
#             if RESET_INT1_TO_ZERO:
#                 if PRINT:
#                     print "===>>> Reset integrator to zero!"
#                 self.int.reset(0.0)
#             else:
#                 if PRINT:
#                     print "est_psi_dot: ", self.est_psi_dot
#                     print "initial integrator output: ", (self.est_psi_dot / self.gain - self.err * self.P)
#                 self.int.reset(self.est_psi_dot / self.gain - self.err * self.P)
#             self.reset_int1 = False
#         if RESET_INT2 and self._i == 1:
#             if PRINT:
#                 print "initial output of integrator two: ", self.err * self.D
#             self.int2.reset((self.err * self.D))
#         # begin interate
#         # print "------------------"
#         if INIT_OPT_TO_ZERO:
#             x = scipy.optimize.broyden1(self._calcResidual, [0.0, 0.0], f_tol=1e-14)
#         else:
#             x = scipy.optimize.broyden1(self._calcResidual, [self.k_u_in, self.k_psi_in], f_tol=1e-14)
#         sat1_in, sat1_out, sat2_in, sat2_out, int_in = self._calcSat1In_Sat1Out_SatIn_Sat2Out(x)
#         self.k_u_in = (sat2_out - sat2_in) / self.ndi_gain
#         self.k_psi_in = sat1_out - sat1_in
#         self.Kpsi_out = (sat1_out - sat1_in) * self.K_psi
#         self.Ku_out = (sat2_out - sat2_in) * self.K_u
#         # print "sat1_in, sat1_out, sat2_in, sat2_out", sat1_in, sat1_out, sat2_in, sat2_out
#         # end first iteration loop
#         self.int_in = int_in
#         if self.psi_dot_set is not None:
#             if USE_RADIUS and self.radius is not None:
#                 self.psi_dot_set_final = self.omega / self.radius # desired turn rate during the turns
#             self.psi_dot_set = self.psi_dot_set * TAU + self.psi_dot_set_final * (1-TAU)

#             self.u_s = saturation(self._linearize(self.psi_dot_set), -1.0, 1.0)
#             # self.err = 0.0
#         else:
#             self.u_s = sat2_out

#         self._i += 1
#         return self.u_s

#     def getErr(self):
#         """ Return the heading/ course error of the controller. """
#         return self.err

#     def getKpsi_out(self):
#         return self.Kpsi_out

#     def getKu_out(self):
#         return self.Ku_out

#     def getIntIn(self):
#         return self.int_in

#     def getIntOut(self):
#         return self.int.getOutput()

#     def getChiFactor(self):
#         return self.chi_factor

#     def onTimer(self):
#         self.int.onTimer()
#         self.int2.onTimer()

#     def calcSteering(self, parking, period_time):
#         self.period_time = period_time
#         self._solve(parking)
#         return self.u_s

#     def getSteering(self):
#         return self.u_s

#     def getState(self):
#         if self.psi_dot_set is not None:
#             turning = True
#             value = self.psi_dot_set
#         else:
#             turning = False
#             value = self.attractor
#         return turning, value

# if __name__ == "__main__":
#     fpc = FlightPathController()
#     u_s = fpc.calcSteering(False, 0.02)
#     print "u_s:", form(u_s)
#     u_d = 0.24
#     v_a = 24.0
#     beta = radians(70.0)
#     psi = radians(90.0)
#     chi = psi
#     phi = 0.0
#     omega = 5.0
#     fpc.onNewEstSysState(phi, beta, psi, chi, omega, v_a, u_d=u_d)
#     fpc.onTimer()
#     u_s = fpc.calcSteering(False, 0.02)
#     print "u_s:", form(u_s)