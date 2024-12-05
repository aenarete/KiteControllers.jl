"""
A collection of control functions for discrete control

Functions:

- saturate
- limit 
- merge_angles
- moving_average

Implemented as described in the PhD thesis of Uwe Fechner.
"""

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
Limit the value of a variable. 

Usage:
@limit x 1 4 # limits the value to the range    1 .. 4,  modifies x
@limit x 10  # limits the value to the range -inf .. 10, modifies x

"""
macro limit(name, min, max=nothing)
    if isnothing(max)
        max = min
        min = :(typemin($min))
    end

    return esc( :($name = clamp($name, $min, $max)) )
end

"""
    merge_angles(alpha, beta, factor_beta)

Calculate the weighted average of two angles. The weight of beta,
factor_beta must be between 0 and 1.
"""
function merge_angles(alpha, beta, factor_beta)
    x1 = sin(alpha)
    y1 = cos(alpha)
    x2 = sin(beta)
    y2 = cos(beta)
    x = x1 * (1.0 - factor_beta) + x2 * factor_beta
    y = y1 * (1.0 - factor_beta) + y2 * factor_beta
    atan(x, y)
end

# calculate the moving average of a vector over a window
function moving_average(data, window)
    local result
    if length(data) <= window
        result = mean(data)
    else
        result = mean(data[length(data)-window:end])
    end
    result
end