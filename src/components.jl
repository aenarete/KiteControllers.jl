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
@with_kw mutable struct Integrator @deftype Float64
    dt                  # timestep [s]
    i           = 1.0   # integration constant
    output      = 0.0
    last_output = 0.0
end

function  Integrator(dt, i=1.0, x0=0.0)
    Integrator(dt, i, x0, x0)
end

function reset(int::Integrator, x0=0.0)
    int.output = x0
    int.last_output = x0
    nothing
end

function calc_output(int::Integrator, input)
    int.output = int.last_output + input * int.i * int.dt
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
# Limit the rate of change of the output signal (return value of calc_output) to ± limit.
# Unit of limit: 1/s
@with_kw mutable struct RateLimiter @deftype Float64
    dt = 0.05
    limit = 1
    output = 0
    last_output = 0
end

function RateLimiter(dt, limit=1.0, x0=0.0)
    RateLimiter(dt, limit, x0, x0)
end

function reset(ud::RateLimiter, x0=0.0)
    ud.output = x0
    ud.last_output = x0
end

function calc_output(rl::RateLimiter, input)
    if input - rl.last_output > rl.limit * rl.dt
        rl.output = rl.last_output + rl.limit * rl.dt
    elseif input - rl.last_output < -rl.limit * rl.dt
        rl.output = rl.last_output - rl.limit * rl.dt
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
# docs/mixer_2ch.png
@with_kw mutable struct Mixer_2CH @deftype Float64
    dt
    t_blend = 1.0
    factor_b = 0
    select_b::Bool = false
end

function Mixer_2CH(dt, t_blend)
    Mixer_2CH(dt, t_blend, 0, false)
end

function select_b(m2::Mixer_2CH, select_b::Bool)
    m2.select_b = select_b
end

function on_timer(m2::Mixer_2CH)
    if m2.select_b
        integrator_in = 1.0 / m2.t_blend
    else
        integrator_in = -1.0 / m2.t_blend
    end
    m2.factor_b += integrator_in * m2.dt
    m2.factor_b = saturate(m2.factor_b, 0, 1)
end

function calc_output(m2::Mixer_2CH, input_a, input_b)
    input_b * m2.factor_b + input_a * (1.0 - m2.factor_b)
end

# Mixer_3CH
# Mix three analog inputs. Implements the simulink block diagram, shown in
# docs/mixer_3ch.png
@with_kw mutable struct Mixer_3CH @deftype Float64
    dt
    t_blend = 1.0
    factor_b = 0
    factor_c = 0
    select_b::Bool = false
    select_c::Bool = false
end

function Mixer_3CH(dt, t_blend = 1.0)
    Mixer_3CH(dt, t_blend, 0, 0, false, false)
end

function on_timer(m3::Mixer_3CH)
    # calc output of integrator b
    if m3.select_b
        integrator_b_in = 1.0 / m3.t_blend
    else
        integrator_b_in = -1.0 / m3.t_blend
    end
    m3.factor_b += integrator_b_in * m3.dt
    m3.factor_b = saturate(m3.factor_b, 0, 1)
    # calc output of integrator c
    if m3.select_c
        integrator_c_in = 1.0 / m3.t_blend
    else
        integrator_c_in = -1.0 / m3.t_blend
    end
    m3.factor_c += integrator_c_in * m3.dt
    m3.factor_c = saturate(m3.factor_c, 0, 1)  
    nothing
end

function select_b(m3::Mixer_3CH, select_b::Bool)
    m3.select_b = select_b
    if select_b
        select_c(m3, false)
    end
end

function select_c(m3::Mixer_3CH, select_c::Bool)
    m3.select_c = select_c
    if select_c
        select_b(m3, false)
    end
end

function calc_output(m3::Mixer_3CH, input_a, input_b, input_c)
    input_b * m3.factor_b + input_c * m3.factor_c + input_a * (1.0 - m3.factor_b - m3.factor_c)
end

"""
Return the controller state as integer.

wcsLowerForceControl = 0 # input b selected 
wcsSpeedControl = 1      # input a selected
wcsUpperForceControl = 2 # input c selected
"""
function get_state(m3::Mixer_3CH)
    Int(! m3.select_b && ! m3.select_c) + 2 * Int(m3.select_c)
end
