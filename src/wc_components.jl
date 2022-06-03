"""
A collection of control functions and components for winch control

Components:

- CalcVSetIn   calculate the set speed of the speed controller, using soft switching
- Winch        model of 20 kW winch, 4000 N max force, 8 m/s max speed
- SpeedController
- LowerForceController
- UpperForceController

Implemented as described in the PhD thesis of Uwe Fechner.
"""

"""
Settings of the WinchController
"""
@with_kw mutable struct WCSettings @deftype Float64
    "timestep of the winch controller"
    dt = 0.05
    fac = 0.25
    eps = 1e-6
    "blending time of the mixers in seconds"
    t_blend = 0.25
    "limitation of the reel-out speed error, used by the input saturation block of the speed controller"
    v_sat_error = 1.0
    "limitation of the reel-out speed , used by the output saturation block of the speed controller"
    v_sat = 12.0 # was: 8.0
    "maximal reel-in speed [m/s]"
    v_ri_max = 12.0
    "P value of the speed controller"
    p_speed = 0.125
    "I value of the speed controller"
    i_speed = 4.0
    "back calculation constant for the anti-windup loop of the speed controller"
    kb_speed = 4.0
    "tracking constant of the speed controller"
    kt_speed = 5.0
    "reel-out velocity where the set force should reach it's maximum"
    vf_max = 2.75
    "P constant of the lower force controller"
    pf_low = 1.44e-4 * fac
    "I constant of the lower force controller"
    if_low = 7.5e-3 * 1.5 * fac
    "back calculation constant for the anti-windup loop of the lower force controller"
    kbf_low = 1.0
    "tracking constant of the lower force controller"
    ktf_low = 8.0
    "lower force limit [N]"
    f_low = 300
    "upper force limit [N]"
    f_high = 4000
    "P constant of upper force controller"
    pf_high = 0.007 * 0.33
    "I constant of upper force controller"
    if_high = 0.003 * 0.33
    "D constant of upper force controller"
    df_high = 2e-5 * 2.0 * 0.0
    "filter constant n of upper force controller"
    nf_high = 15.0
    "back calculation constant for the anti-windup loop of the upper force controller"
    kbf_high = 1.0
    "tracking constant of the upper force controller"
    ktf_high = 10.0
    "interations of the winch model"
    winch_iter = 10
    "maximal acceleration of the winch (derivative of the set value of the reel-out speed)"
    max_acc = 8.0
    "proportional factor of the square root law, see function calc_vro"
    kv = 0.06
end

# Component for calculation v_set_in, using soft switching.
@with_kw mutable struct CalcVSetIn @deftype Float64
    mixer2::Mixer_2CH = Mixer_2CH()
    wcs::WCSettings = WCSettings()
    input_a     = 0
    input_b     = 0
end

function CalcVSetIn(dt, wcs::WCSettings)
    m2 = Mixer_2CH(dt, wcs.t_blend)
    CalcVSetIn(m2, wcs, 0, 0)
end

""" 

Calculate the optimal reel-out speed for a given force. 
"""
function calc_vro(wcs::WCSettings, force; test=false)
    if test
        return sqrt(force) * wcs.kv
    else
        if force >= wcs.f_low
            return wcs.vf_max * sqrt((force - wcs.f_low) / (wcs.f_high - wcs.f_low))
        else
            return -wcs.vf_max * sqrt((wcs.f_low - force) / (wcs.f_high - wcs.f_low))
        end
    end
end

"""
    set_vset_pc(cvi::CalcVSetIn, v_set_pc, force)

Parameters:
- force:    measured tether force [N]
- v_set_pc: only used during manual operation or park-at-length. If it is `nothing`,
            v_set_in is calculated as function of the force.
"""
function set_vset_pc(cvi::CalcVSetIn, v_set_pc, force=nothing)
    if isnothing(v_set_pc)
        cvi.input_a = calc_vro(cvi.wcs, force)
        select_b(cvi.mixer2, false)
    else
        cvi.input_b = v_set_pc
        select_b(cvi.mixer2, true)
    end
    nothing
end

"""
    calc_output(cvi::CalcVSetIn)

Returns v_set_in: Either v_set, or a value, proportional to the sqare root of the force.
"""
function calc_output(cvi::CalcVSetIn)
    calc_output(cvi.mixer2, cvi.input_a, cvi.input_b)
end

function on_timer(cvi::CalcVSetIn)
    on_timer(cvi.mixer2)
end

# Class, that calculates the acceleration of the tether based on the tether force
# and the set speed (= synchronous speed). Asynchronous motor model and drum inertia
# are taken into account.
@with_kw mutable struct Winch @deftype Float64
    wcs::WCSettings = WCSettings()
    wm::AsyncMachine = AsyncMachine()
    v_set     = 0 # input
    force     = 0 # input
    acc       = 0 # output
    speed     = 0 # output; reel-out speed; only state of this model
end

function set_v_set(w::Winch, v_set)
    w.v_set = v_set
end

function set_force(w::Winch, force)
    w.force = force
end

function get_speed(w::Winch) w.speed end
function get_acc(w::Winch) w.acc end

function on_timer(w::Winch)
    acc = 0.0
    for i in 1:w.wcs.winch_iter
        w.acc = calc_acceleration(w.wm, w.v_set, w.speed, w.force)
        acc += w.acc
        w.speed += w.acc * w.wcs.dt/w.wcs.winch_iter
    end
    w.acc = acc/w.wcs.winch_iter
end

# class KiteModel(object):
#     r""" Class, that calculates the position of the kite (elevation angle beta and azimuth angle phi) and the
#     orientation of the kite (psi and psi_dot) as function of:
#     u_s: the relative steering, output of the KCU model (KCU: kite control unit)
#     u_d_prime: the normalized depower settings
#     v_a: the apparent wind speed
#     omega: the angular velocity of the kite
#     implements the simulink diagram ./01_doc/kite_model.png
#     """
#     def __init__(self, beta_0=33.0, psi_0=90.0, phi_0=0.0, K_d_s=1.5, c1=0.262, c2=6.27, omega=0.08):
#         self.int_beta = Integrator(x_0=radians(beta_0)) # integrator output: the elevation angle beta
#         self.int_psi =  Integrator(x_0=radians(psi_0))  # integrator output: heading angle psi, not normalized
#         self.int_phi =  Integrator(x_0=radians(phi_0))  # integrator output: azimuth angle phi
#         self.u_s = 0.0
#         self.v_a = 0.0
#         self.u_d_prime = 0.2
#         self.k_d_s = K_d_s
#         self.omega = omega
#         self.c0 = 0.0
#         self.c1 = c1
#         self.c2 = c2
#         self.psi_dot = 0.0
#         self.a = 0.0
#         self.m1 = self.c2 / 20.0
#         self.res = np.zeros(2)
#         self.psi = radians(psi_0)
#         self.psi_dot = 0.0
#         self.beta = radians(beta_0)
#         self.phi = radians(phi_0)
#         self.x0 = 0.0

#     def setUS(self, u_s):
#         self.u_s = u_s

#     def setUD_prime(self, u_d_prime):
#         self.u_d_prime = u_d_prime

#     def setVA(self, v_a):
#         self.v_a = v_a

#     def setOmega(self, omega):
#         self.omega = omega

#     def getPsiDot(self):
#         return self.psi_dot

#     def getPsi(self):
#         return self.psi
        
#     def getBeta(self):
#         return self.beta
        
#     def getPhi(self):
#         return self.phi
        
#     def getX0(self):
#         return self.x0
        
#     def calcX0_X1_psi_dot(self, x):
#         x0, x1 = x[0], x[1]
#         psi_dot = self.a + self.m1 * sin(x0) * cos(x1)
#         x0 = self.int_psi.calcOutput(psi_dot)
#         x1 = self.int_beta.calcOutput(self.omega * cos(x0))
#         return x0, x1, psi_dot

#     def calcResidual(self, x):
#         x0, x1, psi_dot = self.calcX0_X1_psi_dot(x)
#         self.res[0] = (x0 - x[0]) * 0.5
#         self.res[1] = (x1 - x[1]) 
#         return self.res

#     def solve(self):
#         divisor = self.u_d_prime * self.k_d_s + 1.0
#         assert abs(divisor) > EPSILON
#         self.a = (self.u_s - self.c0) * self.v_a * self.c1 / divisor
#         self.m1 = self.c2 / 20.0
#         x = scipy.optimize.broyden1(self.calcResidual, [self.x0, self.beta], f_tol=1e-14)
#         x0, x1, psi_dot = self.calcX0_X1_psi_dot(x)
#         self.psi_dot = psi_dot
#         self.psi = wrapToPi(x0)
#         self.x0 = x0
#         self.beta = x1
#         self.phi = self.int_phi.calcOutput(-(sin(x0) * self.omega))

#     def onTimer(self):
#         self.int_beta.onTimer()
#         self.int_psi.onTimer()
#         self.int_phi.onTimer()

# class SpeedController(object):
#     """
#     PI controller for the reel-out speed of the winch in speed control mode.
#     While inactive, it tracks the value from the tracking input.
#     Back-calculation is used as anti-windup method and for tracking. The constant for
#     anti-windup is K_b, the constant for tracking K_t
#     Implements the simulink block diagram, shown in ./01_doc/speed_controller.png.
#     """
#     def __init__(self, P=P_SPEED, I=I_SPEED, K_b=kb_speed, K_t=kt_speed):
#         self._P = P
#         self.integrator = Integrator()
#         self._I = I
#         self._K_b = K_b
#         self._K_t = K_t
#         self._v_act = 0.0
#         self._v_set_in = 0.0
#         self._inactive = False
#         self._tracking = 0.0
#         self._v_err = 0.0      # output, calculated by solve
#         self._v_set_out = 0.0  # output, calculated by solve
#         self.limiter = RateLimiter()
#         self.delay = UnitDelay()
#         self.res = np.zeros(2)

#     def setInactive(self, inactive):
#         assert type(inactive) == bool
#         # if it gets activated
#         if self._inactive and not inactive:
#             # print "SC: Reset. v_set: ", self._tracking
#             self.integrator.reset(self._tracking)
#             self.limiter.reset(self._tracking)
#             self._v_set_out = self._tracking
#         self._inactive = inactive

#     def setVAct(self, v_act):
#         self._v_act = v_act

#     def setVSetIn(self, v_set_in):
#         self._v_set_in = v_set_in

#     def setTracking(self, tracking):
#         self._tracking = tracking

#     def calcSat2In_Sat2Out_rateOut(self, x):
#         kb_in = x[0]
#         kt_in = x[1]
#         int_in = self._I * self.sat_out + self._K_b * kb_in + self._K_t * kt_in * (self._inactive)
#         int_out = self.integrator.calcOutput(int_in)
#         if True:
#             sat2_in = int_out + self._P * self.delay.calcOutput(self.sat_out)
#         else:
#             sat2_in = int_out + self._P * self.sat_out
#         sat2_out = saturation(sat2_in, -V_RI_MAX, V_SAT)
#         rate_out = self.limiter.calcOutput(sat2_out)
#         return sat2_in, sat2_out, rate_out, int_in

#     def calcResidual(self, x):
#         """
#         Function, that calculates the residual for the given kb_in and kt_in estimates
#         of the feed-back loop of the integrator.
#         """
#         # TODO: calculate kt_in correctly
#         # kt_in = x[1]
#         sat2_in, sat2_out, rate_out, int_in = self.calcSat2In_Sat2Out_rateOut(x)
#         kt_in = self._tracking - sat2_out
#         kb_in = sat2_out - sat2_in
#         self.res[0] = kb_in - x[0]
#         self.res[1] = kt_in - x[1]
#         # print self.res[0], kb_in
#         return self.res

#     def solve(self):
#         err = self._v_set_in - self._v_act
#         if self._inactive:
#             self._v_err = 0.0
#         else:
#             self._v_err = err
#         self.sat_out = saturation(err, -V_SAT_ERR, V_SAT_ERR)
#         # begin interate
#         # print "------------------"
#         x = scipy.optimize.broyden2(self.calcResidual, [0.0, 0.0], f_tol=1e-14)
#         sat2_in, sat2_out, rate_out, int_in = self.calcSat2In_Sat2Out_rateOut(x)
#         # print "int_in, sat2_in", int_in, sat2_in
#         # end first iteration loop
#         self._v_set_out = rate_out

#     def onTimer(self):
#         self.limiter.onTimer()
#         self.integrator.onTimer()
#         self.delay.onTimer()

#     def getVSetOut(self):
#         self.solve()
#         return self._v_set_out

#     def getVErr(self):
#         if self._inactive:
#             return 0.0
#         else:
#             return self._v_err

# class LowerForceController(object):
#     """
#     PI controller for the lower force of the tether.
#     While inactive, it tracks the value from the tracking input.
#     Back-calculation is used as anti-windup method and for tracking. The constant for
#     anti-windup is K_b, the constant for tracking K_t
#     Implements the simulink block diagram, shown in ./01_doc/lower_force_controller.png.
#     """
#     def __init__(self, P, I, K_b, K_t):
#         self._P = P
#         self.integrator = Integrator()
#         self._I = I
#         self._K_b = K_b
#         self._K_t = K_t
#         self._v_act = 0.0
#         self._force = 0.0
#         self._reset = False
#         self._active = False
#         self._f_set = 0.0
#         self._v_sw = 0.0
#         self._tracking = 0.0
#         self._f_err = 0.0      # output, calculated by solve
#         self._v_set_out = 0.0  # output, calculated by solve
#         self.limiter = RateLimiter()
#         self.delay = UnitDelay()
#         self.res = np.zeros(2)

#     def _set(self):
#         """ internal method to set the SR flip-flop and activate the force controller """
#                 # if it gets activated
#         if self._reset:
#             return
#         if not self._active:
#             # print "Reset. Tracking: ", self._tracking
#             self.integrator.reset(self._tracking)
#             self.limiter.reset(self._tracking)
#             self._v_set_out = self._tracking
#         self._active = True

#     def _updateReset(self):
#         if (self._v_act - self._v_sw) >= 0.0 or self._reset:
#             if (self._force - self._f_set) > 0.0 or self._reset:
#                 self._active = False

#     def setVAct(self, v_act):
#         self._v_act = v_act

#     def setForce(self, force):
#         self._force = force

#     def setReset(self, reset):
#         self._reset = reset
#         self._updateReset()

#     def setFSet(self, f_set):
#         self._f_set = f_set

#     def setV_SW(self, v_sw):
#         self._v_sw = v_sw
#         # print "--->>>", self._v_sw, self._v_act
# #        if self._active and (self._v_act - self._v_sw) >= 0.0:
# #            print "-->", self._v_act

#     def setTracking(self, tracking):
#         self._tracking = tracking

#     def calcSat2In_Sat2Out_rateOut(self, x):
#         kb_in = x[0]
#         kt_in = x[1]
#         int_in = self._I * self._f_err + self._K_b * kb_in + self._K_t * kt_in * (not self._active)
#         int_out = self.integrator.calcOutput(int_in)
#         sat2_in = int_out + self._P * self.delay.calcOutput(self._f_err)
#         sat2_out = saturation(sat2_in, -V_RI_MAX, V_SAT)
#         rate_out = self.limiter.calcOutput(sat2_out)
#         return sat2_in, sat2_out, rate_out, int_in

#     def solve(self):
#         self._updateReset()
#         err = self._force - self._f_set

#         if not self._active:
#             # activate the force controller if the force drops below the set force
#             if err < 0.0:
#                 self._set()
#                 # print "err: ", err
#             self._f_err = 0.0
#         else:
#             self._f_err = err

#         # begin interate
#         # print "------------------"
#         x = scipy.optimize.broyden2(self.calcResidual, [0.0, 0.0], f_tol=1e-14)
#         sat2_in, sat2_out, rate_out, int_in = self.calcSat2In_Sat2Out_rateOut(x)
#         # print "int_in, sat2_in", int_in, sat2_in
#         # end first iteration loop
#         self._v_set_out = rate_out

#     def calcResidual(self, x):
#         """
#         Function, that calculates the residual for the given kb_in and kt_in estimates
#         of the feed-back loop of the integrator.
#         """
#         sat2_in, sat2_out, rate_out, int_in = self.calcSat2In_Sat2Out_rateOut(x)
#         kt_in = self._tracking - sat2_out
#         kb_in = rate_out - sat2_in
#         self.res[0] = kb_in - x[0]
#         self.res[1] = kt_in - x[1]
#         # print self.res[0], kb_in
#         return self.res

#     def getVSetOut(self):
#         self.solve()
#         return self._v_set_out

#     def getFErr(self):
#         return self._f_err

#     def getFSetLow(self):
#         return self._active * self._f_set

#     def onTimer(self):
#         self.limiter.onTimer()
#         self.integrator.onTimer()
#         self.delay.onTimer()

# class UpperForceController(object):
#     """
#     PI controller for the lower force of the tether.
#     While inactive, it tracks the value from the tracking input.
#     Back-calculation is used as anti-windup method and for tracking. The constant for
#     anti-windup is K_b, the constant for tracking K_t
#     Implements the simulink block diagram, shown in ./01_doc/lower_force_controller.png.
#     """
#     def __init__(self, P, I, D, N, K_b, K_t):
#         self._P = P
#         self.integrator = Integrator()
#         self.int2 = Integrator() # integrater of the D part
#         self._I = I
#         self._D = D
#         self._N = N
#         self._K_b = K_b
#         self._K_t = K_t
#         self._v_act = 0.0
#         self._force = 0.0
#         self._reset = False
#         self._active = False
#         self._f_set = 0.0
#         self._v_sw = 0.0
#         self._tracking = 0.0
#         self._f_err = 0.0      # output, calculated by solve
#         self._v_set_out = 0.0  # output, calculated by solve
#         self.limiter = RateLimiter()
#         self.delay = UnitDelay()
#         self.res = np.zeros(3)

#     def _set(self):
#         """ internal method to set the SR flip-flop and activate the force controller """
#                 # if it gets activated
#         if self._reset:
#             return
#         if not self._active:
#             # print "Reset. Tracking: ", self._tracking
#             self.integrator.reset(self._tracking)
#             self.int2.reset(0.0)
#             self.limiter.reset(self._tracking)
#             self._v_set_out = self._tracking
#         self._active = True

#     def _updateReset(self):
#         if (self._v_act - self._v_sw) <= 0.0 or self._reset:
#             if (self._force - self._f_set) < 0.0 or self._reset:
#                 self._active = False

#     def setVAct(self, v_act):
#         self._v_act = v_act

#     def setForce(self, force):
#         self._force = force

#     def setReset(self, reset):
#         self._reset = reset

#     def setFSet(self, f_set):
#         self._f_set = f_set

#     def setV_SW(self, v_sw):
#         self._v_sw = v_sw
#         # print "--->>>", self._v_sw, self._v_act
# #        if self._active and (self._v_act - self._v_sw) >= 0.0:
# #            print "-->", self._v_act

#     def setTracking(self, tracking):
#         self._tracking = tracking

#     def calcSat2In_Sat2Out_rateOut(self, x):
#         kb_in = x[0]
#         kt_in = x[1]
#         int2_in = x[2]
#         int_in = self._I * self._f_err + self._K_b * kb_in + self._K_t * kt_in * (not self._active)
#         int_out = self.integrator.calcOutput(int_in)
#         int2_out = self.int2.calcOutput(int2_in)
#         sat2_in = int_out + self._P * self.delay.calcOutput(self._f_err) + self._N * (self._f_err * self._D - int2_out)

#         sat2_out = saturation(sat2_in, -V_RI_MAX, V_SAT)
#         rate_out = self.limiter.calcOutput(sat2_out)
#         return sat2_in, sat2_out, rate_out, int_in, int2_in

#     def solve(self):
#         self._updateReset()
#         err = self._force - self._f_set

#         if not self._active:
#             # activate the force controller if the force rises above the set force
#             if err >= 0.0:
#                 self._set()
#                 # print "err: ", err
#             self._f_err = 0.0
#         else:
#             self._f_err = err
#         # begin interate
#         # print "------------------"
#         x = scipy.optimize.broyden1(self.calcResidual, [0.0, 0.0, 0.0], f_tol=1e-14)
#         sat2_in, sat2_out, rate_out, int_in, int2_in = self.calcSat2In_Sat2Out_rateOut(x)
#         # print "int_in, sat2_in", int_in, sat2_in
#         # end first iteration loop
#         self._v_set_out = rate_out

#     def calcResidual(self, x):
#         """
#         Function, that calculates the residual for the given kb_in and kt_in estimates
#         of the feed-back loop of the integrator.
#         """
#         sat2_in, sat2_out, rate_out, int_in, int2_in = self.calcSat2In_Sat2Out_rateOut(x)
#         kt_in = self._tracking - sat2_out
#         kb_in = rate_out - sat2_in
#         self.res[0] = kb_in - x[0]
#         self.res[1] = kt_in - x[1]
#         self.res[2] = int2_in - x[2]
#         # print self.res[0], kb_in
#         return self.res

#     def getVSetOut(self):
#         self.solve()
#         return self._v_set_out

#     def getFErr(self):
#         return self._f_err

#     def getFSetUpper(self):
#         return self._active * self._f_set

#     def onTimer(self):
#         self.limiter.onTimer()
#         self.integrator.onTimer()
#         self.int2.onTimer()
#         self.delay.onTimer()

# if __name__ == "__main__":
#     limiter = RateLimiter()
#     mix2 = Mixer_2CH()
#     mix3 = Mixer_3CH()
#     pid1 = SpeedController()
#     pid2 = LowerForceController(pf_low, if_low, kbf_low, ktf_low)
#     pid3 = UpperForceController(pf_high, if_high, df_high, nf_high, kbf_high, ktf_high)
#     winch = Winch()
#     kite = KiteModel()
