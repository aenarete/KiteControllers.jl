# Component, that calculates the position of the kite (elevation angle beta and azimuth angle phi) and the
# orientation of the kite (psi and psi_dot) as function of:
# u_s: the relative steering, output of the KCU model (KCU: kite control unit)
# u_d_prime: the normalized depower settings
# v_a: the apparent wind speed
# omega: the angular velocity of the kite
# implements the simulink diagram docs/kite_model.png

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
        
function calc_x0_x1_psi_dot(km::KiteModel, x)
    x0, x1 = x[1], x[2]
    psi_dot = km.a + km.m1 * sin(x0) * cos(x1)
    x0 = calc_output(km.int_psi, psi_dot)             # self.int_psi.calcOutput(psi_dot)
    x1 = calc_output(km.int_beta, km.omega * cos(x0)) # self.int_beta.calcOutput(self.omega * cos(x0))
    x0, x1, psi_dot
end

function solve(km::KiteModel)
    function residual(F, x)
        x0, x1, psi_dot = calc_x0_x1_psi_dot(km, x)
        F[begin]   = (x0 - x[begin]) * 0.5
        F[begin+1] = (x1 - x[begin+1])
    end
    divisor = km.u_d_prime * km.k_d_s + 1.0
    @assert abs(divisor) > EPSILON
    km.a = (km.u_s - km.c0) * km.v_a * km.c1 / divisor
    km.m1 = km.c2 / 20.0
    res = nlsolve(residual!, [ km.x0; km.beta], ftol=1e-14)
    @assert converged(res)
    x = res.zero
    x0, x1, psi_dot = calc_x0_x1_psi_dot(km, x)
    km.psi_dot = psi_dot
    km.psi = wrap_to_pi(x0)
    km.x0 = x0
    km.beta = x1
    km.phi = calc_output(km.int_phi, -(sin(x0) * km.omega))
end

function on_timer(km::KiteModel)
    on_timer(km.int_beta)
    on_timer(km.int_psi)
    on_timer(km.int_phi)
end
