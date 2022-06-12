# """ Module, that provides a kite estimator for the turn rate of the kite. """

# from Settings import PERIOD_TIME, pro
# from cgkit.cgtypes import vec3
# from math import pi, sin, cos, acos, copysign, radians, sqrt
# import numpy as np

# SIGMA = 1e-3   # if the norm of the velocity is beyond this value, it is assumed to be zero

# def calcTangentialVel(pos_kite, vel_kite):
#     """ Calculate the tangential velocity of the kite with respect to the sphere with the
#     diameter of the kite distance. Input parameters must be of type vec3. """
#     v_radial = vel_kite * pos_kite / pos_kite.length()
#     v_ortho = vel_kite - v_radial * pos_kite.normalize()
#     return v_ortho

# def calcTangentialVelNorm(pos_kite, vel_kite):
#     return abs(calcTangentialVel(pos_kite, vel_kite))

# def turn(pos_kite, vel_kite, turn_angle):
#     """ Turn the velocity vector around the position vector by the turn_angle.
#     See: http://inside.mines.edu/~gmurray/ArbitraryAxisRotation/ """
#     x, y, z = vel_kite[0], vel_kite[1], vel_kite[2]
#     axis = pos_kite.normalize()
#     u, v, w = axis[0], axis[1], axis[2]
#     si, co = sin(turn_angle), cos(turn_angle)
#     term = (-u*x - v*y - w*z) * (1.0 - co)
#     res0 = -u * term + x*co + (-w*y + v*z) * si
#     res1 = -v * term + y*co + ( w*x - u*z) * si
#     res2 = -w * term + z*co + (-v*x + u*y) * si
#     return vec3(res0, res1, res2)
    
# class KiteEstimator(object):
#     """ Estimate the turn rate of the kite. """
#     def __init__(self):
#         self.last_time    = - PERIOD_TIME       # rel_time when calcPosPredicted was called the last time
#         self.last_v_t     = vec3(0.0, 0.0, 0.0) # last tangential velocity (ENU coordinates)
#         self.last_v_theta = 0.0                 # inital turn rate         (radian per second)
#         self.v_wk_av = None                     # average wind speed at the height of the kite
#         self.alpha_av = None            

#     def getTurnRate(self):
#         return self.last_v_theta

#     def estTheta(self, rel_time, pos_kite, vel_kite, v_reel_out):
#         """ Estimate the current turn rate, assuming that the kite is at the position pos_kite,
#         flying with the given speed v_kite, operating at a winch with the reel-out speed v_reel_out.

#         Input Types:
#         rel_time, v_reel_out: real;
#         pos_kite, vel_kite: vec3

#         Returns:
#         v_theta # the turn rate of the kite
#         """

#         # calculate the turn rate
#         delta_t = rel_time - self.last_time
#         self.last_time = rel_time
#         v_t = calcTangentialVel(pos_kite, vel_kite)
#         if self.last_v_t.length() > SIGMA and v_t.length() > SIGMA:
#             theta = acos(self.last_v_t.normalize() * v_t.normalize())
#             cross = self.last_v_t.cross(v_t)
#             theta = copysign(theta, pos_kite * cross)
#         else:
#             theta = 0.0
#         if delta_t > 0.01:
#             v_theta = theta / delta_t # rotational rate
#         else:
#             v_theta = self.last_v_theta
#         self.last_v_theta = v_theta
#         self.last_v_t = v_t
#         return v_theta

# if __name__ == "__main__":
#     # simple test of the function 'turn'
#     pos_kite = vec3(0.0, 0.0, 1.0)
#     vel_kite = vec3(0.0, 0.1, 0.0)
#     print vel_kite.length()
#     turn_angle = 90.0 * pi / 180.0
#     vel2 = turn(pos_kite, vel_kite, turn_angle)
#     print vel2.length()
#     print vel2
