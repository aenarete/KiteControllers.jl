 """
A collection of control functions and control components for discrete control

Functions:

- saturation
- wrap2pi

Components:

- Integrator

Implemented as described in the PhD thesis of Uwe Fechner.
"""

# discrete integrator with external reset
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


""" Calculate a saturated value, that stays within the given limits. """
function saturate(value, min_, max_)
    result = value
    if result > max_
        result = max_
    elseif result < min_
        result = min_
    end
    result
end

"""
    wrap2pi(angle)

Convert an angle in an infinite range to the range from -pi to pi
"""
function wrap2pi(angle)
    num2pi = floor(angle / 2π + 0.5)
    angle - 2π * num2pi
end

# def mergeAngles(alpha, beta, factor_beta):
#     """
#     Calculate the weighted average of two angles. The weight of beta,
#     factor_beta must be between 0 and 1.
#     """
#     x1 = sin(alpha)
#     y1 = cos(alpha)
#     x2 = sin(beta)
#     y2 = cos(beta)
#     x = x1 * (1.0 - factor_beta) + x2 * factor_beta
#     y = y1 * (1.0 - factor_beta) + y2 * factor_beta
#     return atan2(x, y)

