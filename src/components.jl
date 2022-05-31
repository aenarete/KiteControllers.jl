"""
A collection of generic control components

Components:

- Integrator
- UnitDelay
- RateLimiter
- Mixer_2CH    two channel mixer
- Mixer_3CH    three channel mixer
"""

# Discrete integrator with external reset
@with_kw mutable struct Integrator
    output::Float64      = 0.0
    last_output::Float64 = 0.0
    I::Float64           = 1.0 # integration constant
end

function  Integrator(I, x0=0.0)
    int = Integrator()
    int.output = x0
    int.last_output = x0
    int.I = I
    int
end

function reset(int::Integrator, x0=0.0)
    int.output = x0
    int.last_output = x0
    nothing
end

function update(int::Integrator, input, dt)
    int.output = int.last_output + input * int.I * dt
end

function on_timer(int::Integrator)
    int.last_output = int.output
    nothing
end

# UnitDelay, delay the input signal by one time step.
@with_kw mutable struct UnitDelay @deftype Float64
    last_output = 0
    last_input = 0
end

function calc_output(ud::UnitDelay, input)
    ud.last_input = input
    ud.last_output
end

function on_timer(ud::UnitDelay)
    ud.last_output = ud.last_input
end

function reset(ud::UnitDelay)
    ud.last_input = 0.0
    ud.last_output = 0.0
end

# RateLimiter
# Limit the chate of the output signal per tick (return value of calc_output) to ± limit.
@with_kw mutable struct RateLimiter @deftype Float64
    limit = 1
    output = 0
    last_output = 0
end

function RateLimiter(limit, x0=0.0)
    RateLimiter(limit, x0, x0)
end

function reset(ud::RateLimiter, x0=0.0)
    ud.output = x0
    ud.last_output = x0
end

function calc_output(rl::RateLimiter, input)
    if input - rl.last_output > rl.limit
        rl.output = rl.last_output + rl.limit
    elseif input - rl.last_output < -rl.limit 
        rl.output = rl.last_output - rl.limit
    else
        rl.output = input
    end
    rl.output
end

function on_timer(rl::RateLimiter)
    rl.last_output = rl.output
end

# Mixer_2CH
# Mix two analog inputs. Implements the simulink block diagram, shown in
# ./01_doc/mixer_2ch.png
@with_kw mutable struct Mixer_2CH @deftype Float64
    dt = 0.05
    t_blend = 1.0
    factor_b = 0
    select_b::Bool = false
end

function Mixer_2CH(dt=0.05, t_blend = 1.0)
    Mixer_2CH(dt, t_blend, 0, false)
end

function select_b(m2::Mixer_2CH, select_b)
    ms.select_b = select_b
end

function on_timer(m2::Mixer_2CH)
    if m2.select_b
        integrator_in = 1.0 / m2.t_blend
    else
        integrator_in = -1.0 / m2.t_blend
    end
    m2.factor_b += integrator_in * m2.dt
    if ms.factor_b > 1.0
        ms.factor_b = 1.0
    elseif ms.factor_b < 0
        ms.factor_b = 0
    end
end

function calc_output(m2::Mixer_2CH, input_a, input_b)
    input_b * m2.factor_b + input_a * (1.0 - m2.factor_b)
end

# Mixer_3CH
# Mix three analog inputs. Implements the simulink block diagram, shown in
# ./01_doc/mixer_3ch.png

# class Mixer_3CH(object):
#     def __init__(self):
#         self._factor_b = 0.0
#         self._factor_c = 0.0
#         self._select_b = False
#         self._select_c = False

#     def onTimer(self):
#         """ Must be called every period time. """
#         # calc output of integrator b
#         if self._select_b:
#             integrator_b_in = 1.0 / T_BLEND
#         else:
#             integrator_b_in = -1.0 / T_BLEND
#         self._factor_b += integrator_b_in * PERIOD_TIME
#         if self._factor_b > 1.0:
#             self._factor_b = 1.0
#         if self._factor_b < 0.0:
#             self._factor_b = 0.0
#         # calc output of integrator c
#         if self._select_c:
#             integrator_c_in = 1.0 / T_BLEND
#         else:
#             integrator_c_in = -1.0 / T_BLEND
#         self._factor_c += integrator_c_in * PERIOD_TIME
#         if self._factor_c > 1.0:
#             self._factor_c = 1.0
#         if self._factor_c < 0.0:
#             self._factor_c = 0.0

#     def select_b(self, select_b):
#         assert type(select_b) == bool
#         self._select_b = select_b
#         if select_b:
#             self.select_c(False)

#     def select_c(self, select_c):
#         assert type(select_c) == bool
#         self._select_c = select_c
#         if select_c:
#             self.select_b(False)

#     def calc_output(self, input_a, input_b, input_c):
#         result = input_b * self._factor_b + input_c * self._factor_c \
#                  + input_a * (1.0 - self._factor_b - self._factor_c)
#         return result

#     def get_state(self):
#         """
#         Return the controller state as integer.
#         wcsLowerForceControl = 0
#         wcsSpeedControl = 1
#         wcsUpperForceControl = 2
#         """
#         return (not self._select_b) and (not self._select_c) + 2 * self._select_c
