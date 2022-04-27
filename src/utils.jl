"""
A collection of control functions and control components for discrete control

Functions:

- saturation
- wrap2pi

Components:

- Integrator

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
    wrap2pi(angle)

Convert an angle in an infinite range to the range from -pi to pi
"""
function wrap2pi(angle)
    num2pi = floor(angle / 2π + 0.5)
    angle - 2π * num2pi
end

# class Integrator(object):
#     """ Discrete integrator with external reset. """
#     def __init__(self, I=1.0, x_0=0.0):
#         """
#         Constructor. Parameters:
#         I:   integration constant
#         x_0: initial ouput
#         """
#         self._output = x_0
#         self._last_output = x_0
#         self._I = I

#     def reset(self, x_0):
#         self._output = x_0
#         self._last_output = x_0

#     # TODO: pass period time to calcOutput    
#     def calcOutput(self, input_):
#         self._output = self._last_output + input_ * self._I * PERIOD_TIME
#         return self._output
        
#     def getOutput(self):
#         return self._output
        
#     def getLastOutput(self):
#         return self._last_output
        
#     def onTimer(self):
#         self._last_output = self._output

