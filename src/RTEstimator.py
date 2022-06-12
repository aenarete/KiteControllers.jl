# """ Module, that provides a kite-state estimator class for real-time
# applications. """

# from Settings import PERIOD_TIME, M_DYNEEMA, KCU_MASS, pro
# from cgkit.cgtypes import vec3
# from math import pi, sin, cos, acos, copysign, radians, sqrt
# from AeroBasic import calcCL# calcCL_CD_spline_
# import numpy as np

# D_TETHER = 4e-3

# STEP_SIZE = 10 # one predicted point for each 10 * 50ms
# DELTA_T1  = PERIOD_TIME * STEP_SIZE
# SIGMA = 1e-3   # if the norm of the velocity is beyond this value, it is assumed to be zero
# TETHER_MASS_PER_M = 0.5 * M_DYNEEMA * (D_TETHER/2.0)**2 * pi
# print TETHER_MASS_PER_M
# C_D_TETHER = 0.958
# G = 9.81 # earth acceleration
# LIFT_CORRECTION = 1.03
# K = 0.5 # filter constant for the low pass filter of the wind speed at the height of the kite

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
    
# class Filter(object):
#     """ This class implements a moving average filter.  """
#     def __init__(self, depth = 25):        
#         """ Create an instance of the Filter class. The parameter depth determines the depth of the filter. """
#         self.values = np.zeros(depth) * np.nan    
#         self.index = 0
#         self.depth = depth
    
#     def calcAverage(self, value):
#         if value is None:
#             value = np.nan
#         self.values[self.index] = value
#         self.index += 1
#         if self.index >= self.depth:
#             self.index = 0
#         return np.nanmean(self.values)
        

# class RTEstimator(object):
#     """ Estimate the state of the kite power system. Provide a method to predict the flight path. """

#     def __init__(self):
#         self.last_time    = - PERIOD_TIME       # rel_time when calcPosPredicted was called the last time
#         self.last_v_t     = vec3(0.0, 0.0, 0.0) # last tangential velocity (ENU coordinates)
#         self.last_v_theta = 0.0                 # inital turn rate         (radian per second)
#         self.pos     = vec3(0.0, 0.0, 0.0)      # estimated kite position
#         self.vel     = vec3(0.0, 0.0, 0.0)      # estimated kite velocity
#         self.v_reel_out = 0.0                   # estimated derivative of the kite distance
#         self.v_wk_av = None                     # average wind speed at the height of the kite
#         self.alpha_av = None            
#         self.predicted_reel_out_speed = Filter() # a filter object for the predicted reel out speed

#     def predictFlightPath(self, time_horizon=4.0):
#         """ Calculate the predicted kite path, assuming that the kite is at the position pos_kite,
#         flying with the given speed v_kite, operating at a winch with the reel-out speed v_reel_out
#         for the given time_horizon [s]. """
#         res = []         # create the resulting list of positions
#         pos = self.pos   # starting point for the prediction
#         vel = self.vel
#         res.append(vec3(pos))
#         prediction_steps = int(time_horizon / DELTA_T1)
#         for steps in range (prediction_steps):
#             if pos.length() > SIGMA:
#                 pos = (pos + vel * DELTA_T1).normalize() * (pos.length() + DELTA_T1 * self.v_reel_out)
#             # calculate the new velocity vector by turning it
#             vel = turn(pos, vel, self.last_v_theta * DELTA_T1)
#             # realign it tangential to the sphere
#             vel = calcTangentialVel(pos, vel)
#             res.append(vec3(pos))
#         return res

#     def getTurnRate(self):
#         return self.last_v_theta

#     def estKiteState(self, rel_time, pos_kite, vel_kite, v_reel_out):
#         """ Estimate the current kite state, assuming that the kite is at the position pos_kite,
#         flying with the given speed v_kite, operating at a winch with the reel-out speed v_reel_out.

#         Input Types:
#         rel_time, v_reel_out, time_horizon: real;
#         pos_kite, v_kite: vec3

#         Result:
#         self.last_v_theta # the turn rate of the kite
#         """

#         # calculate the turn rate
#         delta_t = rel_time - self.last_time
#         self.last_time = rel_time
#         # if delta_t > 0.051 or delta_t < 0.049:
#         #    print 'delta_t', delta_t
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
#         # TODO: implement real estimation of position, velocity and kite distance
#         self.pos = pos_kite          # estimate the position
#         self.vel = vel_kite          # estimate the velocity
#         self.v_reel_out = v_reel_out # estimate the derivative of the kite distance
#         # print v_theta * 180.0 / pi

#     def predictedReelOutSpeed(self, force, tether_length, elevation, azimuth, course, alpha, LoD, v_wk, v_tau, rho):
#         """ Estimate the reel-out speed, required for the given force, tether_length,
#         elevation (angle in radian), azimuth angle in wind reference frame,
#         angle of attack alpha (rad), wind speed at the height of the kite v_wk,
#         the norm of the tangential kite velocity v_tau and the air density at the
#         height of the kite rho"""
#         if self.v_wk_av is None:
#             self.v_wk_av = v_wk
#         else:
#             self.v_wk_av = self.v_wk_av * K + v_wk * (1.0 - K)
#         if self.alpha_av is None:
#             self.alpha_av = alpha
#         else:
#             self.alpha_av = self.alpha_av * K + alpha * (1.0 - K)
#         # print "self.v_wk_av, v_wk: ", self.v_wk_av, v_wk
#         try:
#             # 1. calculate the tether force at the kite
#             force_kite=force + tether_length * TETHER_MASS_PER_M * G * sin(elevation)
#             # print 'force_kite: ', force_kite
#             # 2. calculate the aerodynamic force at the kite
#             force_ax=force_kite * cos(elevation)
#             force_az=force_kite * sin(elevation) + G * (pro._kite.mass + KCU_MASS)
#             force_a = sqrt(force_ax * force_ax + force_az * force_az)
#             # print 'aerodynamic force: ', force_a
#             # 3. calculatet cl and cd
#             cl = calcCL(self.alpha_av)
#             # cl, cd_kite = calcCL_CD_spline_(self.alpha_av)
#             cd_kite = cl/LoD
#             cd_tether = 0.31 * tether_length * D_TETHER/pro._kite.area * C_D_TETHER
#             cd = cd_tether + cd_kite
#             cl *= LIFT_CORRECTION
#             # 4. calculated the lift
#             lift = force_a/sqrt(1.0 + (cd*cd/(cl*cl)))
#             # 5. calc theta
#             theta = pi/2.0 - elevation
#             phi   = azimuth
#             chi = course + pi
#             if chi > 2 * pi:
#                 chi -= 2 * pi
#             term0 = self.v_wk_av * sin(theta) * cos(phi)
#             term1 = -(v_tau * sin(chi) + self.v_wk_av * sin(phi))**2
#             term2 = -(v_tau * cos(chi) - self.v_wk_av * cos(phi) * cos(theta))**2
#             term3 = 2 * lift / (cl * pro._kite.area * rho)
#             v_tb = term0 - sqrt(term1 + term2 + term3)
#         except:
#             v_tb = np.nan
#         result = self.predicted_reel_out_speed.calcAverage(v_tb) # filter the output with a 20 sample moving average
#         if np.isnan(result):
#             result = None
#         return result

# if __name__ == "__main__":
#     # simple test of the function 'turn'
#     pos_kite = vec3(0.0, 0.0, 1.0)
#     vel_kite = vec3(0.0, 0.1, 0.0)
#     print vel_kite.length()
#     turn_angle = 90.0 * pi / 180.0
#     vel2 = turn(pos_kite, vel_kite, turn_angle)
#     print vel2.length()
#     print vel2
#     force = 1600.0
#     tether_length = 300.0
#     elevation = radians(20.0)
#     azimuth = 0.0
#     course = 0.0
#     est = RTEstimator()
#     alpha = radians(5.0)
#     v_wk = 11.0
#     v_tau = 25.0
#     rho = 1.0
#     LoD = 4.5
#     v_ro = est.predictedReelOutSpeed(force, tether_length, elevation, azimuth, course, alpha, LoD, v_wk, v_tau, rho)
#     print "Estimated reel-out speed: ", v_ro
    
#     print "kite mass: ", pro._kite.mass
#     print "kite area: ", pro._kite.area
