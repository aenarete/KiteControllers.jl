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


# Component for calculation v_set_in, using soft switching.
@with_kw mutable struct CalcVSetIn @deftype Float64
    wcs::WCSettings
    mixer2::Mixer_2CH = Mixer_2CH(wcs.dt, wcs.t_blend)
    input_a     = 0
    input_b     = 0
end

function CalcVSetIn(wcs::WCSettings)
    CalcVSetIn(wcs=wcs)
end

""" 

Calculate the optimal reel-out speed for a given force. 
"""
function calc_vro(wcs::WCSettings, force)
    if wcs.test
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
    wcs::WCSettings
    wm::AsyncMachine = AsyncMachine()
    v_set     = 0 # input
    force     = 0 # input
    acc       = 0 # output
    speed     = 0 # output; reel-out speed; only state of this model
end
function Winch(wcs::WCSettings)
    Winch(wcs=wcs)
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

# PI controller for the reel-out speed of the winch in speed control mode.
# While inactive, it tracks the value from the tracking input.
# Back-calculation is used as anti-windup method and for tracking. The constant for
# anti-windup is K_b, the constant for tracking K_t
# Implements the simulink block diagram, shown in docs/speed_controller.png.

@with_kw mutable struct SpeedController @deftype Float64
    wcs::WCSettings
    integrator::Integrator = Integrator(wcs.dt)
    limiter::RateLimiter = RateLimiter(wcs.dt, wcs.max_acc)
    delay::UnitDelay = UnitDelay()
    v_act = 0
    v_set_in = 0
    inactive::Bool = false
    tracking = 0
    v_err = 0         # output, calculated by solve
    v_set_out = 0     # output, calculated by solve
    sat_out = 0       # output of saturate block
    res::MVector{2, Float64} = zeros(2)
end
function SpeedController(wcs::WCSettings)
    SpeedController(wcs=wcs)
end

function set_inactive(sc::SpeedController, inactive::Bool)
    # when it gets activated
    if sc.inactive && ! inactive
        reset(sc.integrator, sc.tracking)
        reset(sc.limiter, sc.tracking)
        sc.v_set_out = sc.tracking
    end
    sc.inactive = inactive
end

function set_v_act(sc::SpeedController, v_act)
    sc.v_act = v_act
end

function set_v_set(sc::SpeedController, v_set)
    reset(sc.integrator, v_set)
end

function set_v_set_in(sc::SpeedController, v_set_in)
    sc.v_set_in = v_set_in
end

function set_tracking(sc::SpeedController, tracking)
    sc.tracking = tracking
end

function calc_sat2in_sat2out_rateout_intin(sc::SpeedController, x)
    kb_in = x[begin]
    kt_in = x[begin+1]
    int_in = sc.wcs.i_speed * sc.sat_out + sc.wcs.kb_speed * kb_in + sc.wcs.kt_speed * kt_in * sc.inactive
    int_out = calc_output(sc.integrator, int_in)
    sat2_in = int_out + sc.wcs.p_speed * calc_output(sc.delay, sc.sat_out)
    sat2_out = saturate(sat2_in, -sc.wcs.v_ri_max, sc.wcs.v_sat)
    rate_out = calc_output(sc.limiter, sat2_out)
    sat2_in, sat2_out, rate_out, int_in
end

function solve(sc::SpeedController)
    # Function, that calculates the residual for the given kb_in and kt_in estimates
    # of the feed-back loop of the integrator.
    function calc_residual!(F, x)
        sat2_in, sat2_out, rate_out, int_in = calc_sat2in_sat2out_rateout_intin(sc, x)
        kt_in = sc.tracking - sat2_out
        kb_in = sat2_out - sat2_in
        sc.res[begin]   = kb_in - x[begin]
        sc.res[begin+1] = kt_in - x[begin+1]
        F .= sc.res 
    end

    err = sc.v_set_in - sc.v_act
    if sc.inactive
        sc.v_err = 0.0
    else
        sc.v_err = err
    end
    sc.sat_out = saturate(err, -sc.wcs.v_sat_error, sc.wcs.v_sat_error)
    # begin interate
    sol = nlsolve(calc_residual!, [ 0.0; 0.0], iterations=sc.wcs.max_iter)
    @assert sol.f_converged
    sc.wcs.iter = max(sol.iterations, sc.wcs.iter)
    sat2_in, sat2_out, rate_out, int_in = calc_sat2in_sat2out_rateout_intin(sc, sol.zero)
    sc.v_set_out = rate_out
end

function on_timer(sc::SpeedController)
    on_timer(sc.limiter)
    on_timer(sc.integrator)
    on_timer(sc.delay)
end

function get_v_set_out(sc::SpeedController)
    solve(sc)
end

function get_v_error(sc::SpeedController)
    if sc.inactive
        return 0.0
    else
        return sc.v_err
    end
end

# PI controller for the lower force of the tether.
# While inactive, it tracks the value from the tracking input.
# Back-calculation is used as anti-windup method and for tracking. The constant for
# anti-windup is K_b, the constant for tracking K_t
# Implements the simulink block diagram, shown in docs/lower_force_controller.png.
@with_kw mutable struct LowerForceController <: AbstractForceController @deftype Float64
    wcs::WCSettings
    integrator::Integrator = Integrator(wcs.dt)
    limiter::RateLimiter = RateLimiter(wcs.dt, wcs.max_acc)
    delay::UnitDelay = UnitDelay()
    reset::Bool = false
    active::Bool = false
    force = 0
    f_set = 0
    v_sw = 0
    v_act = 0
    tracking = 0
    f_err = 0         # output, calculated by solve
    v_set_out = 0     # output, calculated by solve
    sat_out = 0       # output of saturate block
    res::MVector{2, Float64} = zeros(2)
end

function LowerForceController(wcs::WCSettings)
    LowerForceController(wcs=wcs)
end

# internal method to set the SR flip-flop and activate the force controller
function _set(lfc::LowerForceController)
    if lfc.reset return end
    if ! lfc.active
       reset(lfc.integrator, lfc.tracking)
       reset(lfc.limiter, lfc.tracking)
       lfc.v_set_out = lfc.tracking
    end
    lfc.active = true
end

function _update_reset(fc::AFC)
    if (fc.v_act - fc.v_sw) >= 0.0 || fc.reset
        if (fc.force - fc.f_set) > 0.0 || fc.reset
            fc.active = false
        end
    end
end

function set_v_act(fc::AFC, v_act)
    fc.v_act = v_act
end

function set_force(fc::AFC, force)
    fc.force = force
end

function set_reset(fc::AFC, reset)
    fc.reset = reset
    _update_reset(fc)
end

function set_f_set(fc::AFC, f_set)
    fc.f_set = f_set
end

function set_v_sw(fc::AFC, v_sw)
    fc.v_sw = v_sw
end

function set_tracking(fc::AFC, tracking)
    fc.tracking = tracking
end

function calc_sat2in_sat2out_rateout_intin(lfc::LowerForceController, x)
    kb_in = x[begin]
    kt_in = x[begin+1]
    int_in = if_low_scaled(lfc.wcs) * lfc.f_err + lfc.wcs.kbf_low * kb_in + lfc.wcs.ktf_low * kt_in * (! lfc.active)
    int_out = calc_output(lfc.integrator, int_in)
    sat2_in = int_out + pf_low_scaled(lfc.wcs) * calc_output(lfc.delay, lfc.f_err)
    sat2_out = saturate(sat2_in, -lfc.wcs.v_ri_max, lfc.wcs.v_sat)
    rate_out = calc_output(lfc.limiter, sat2_out)
    sat2_in, sat2_out, rate_out, int_in
end

function solve(lfc::LowerForceController)
    # Function, that calculates the residual for the given kb_in and kt_in estimates
    # of the feed-back loop of the integrator.
    function calc_residual!(F, x)
        sat2_in, sat2_out, rate_out, int_in = calc_sat2in_sat2out_rateout_intin(lfc, x)
        kt_in = lfc.tracking - sat2_out
        kb_in = rate_out - sat2_in
        lfc.res[begin]   = kb_in - x[begin]
        lfc.res[begin+1] = kt_in - x[begin+1]
        F .= lfc.res 
    end

    _update_reset(lfc)
    err = lfc.force - lfc.f_set
    if ! lfc.active
        # activate the force controller if the force drops below the set force
        if err < 0.0
            _set(lfc)
        end
        lfc.f_err = 0.0
    else
        lfc.f_err = err
    end
    sol = nlsolve(calc_residual!, [ 0.0; 0.0], iterations=lfc.wcs.max_iter)
    @assert sol.f_converged
    lfc.wcs.iter = max(sol.iterations, lfc.wcs.iter)
    sat2_in, sat2_out, rate_out, int_in = calc_sat2in_sat2out_rateout_intin(lfc, sol.zero)
    lfc.v_set_out = rate_out
end

function get_v_set_out(fc::AFC)
    solve(fc)
    fc.v_set_out
end

function get_f_err(fc::AFC)
    fc.f_err
end

function get_f_set_low(lfc::LowerForceController)
    lfc.active * lfc.f_set
end

function on_timer(lfc::LowerForceController)
    on_timer(lfc.limiter)
    on_timer(lfc.integrator)
    on_timer(lfc.delay)
end

# PID controller for the upper force of the tether.
# While inactive, it tracks the value from the tracking input.
# Back-calculation is used as anti-windup method and for tracking. The constant for
# anti-windup is K_b, the constant for tracking K_t
# Implements the simulink block diagram, shown in docs/upper_force_controller.png.
@with_kw mutable struct UpperForceController <: AbstractForceController @deftype Float64
    wcs::WCSettings
    integrator::Integrator = Integrator(wcs.dt)
    int2::Integrator = Integrator(wcs.dt)
    limiter::RateLimiter = RateLimiter(wcs.dt, wcs.max_acc)
    delay::UnitDelay = UnitDelay()
    reset::Bool = false
    active::Bool = false
    f_set = 0
    v_sw = 0
    v_act = 0
    force = 0
    tracking = 0
    f_err = 0         # output, calculated by solve
    v_set_out = 0     # output, calculated by solve
    sat_out = 0       # output of saturate block
    res::MVector{3, Float64} = zeros(3)
end

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

#     def getFSetUpper(self):
#         return self._active * self._f_set

function on_timer(ufc::UpperForceController)
    on_timer(ufc.limiter)
    on_timer(ufc.integrator)
    on_timer(ufc.int2)
    on_timer(ufc.delay)
end
