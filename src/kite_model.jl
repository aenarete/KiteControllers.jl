# Component, that calculates the position of the kite (elevation angle beta and azimuth angle phi) and the
# orientation of the kite (psi and psi_dot) as function of:
# u_s: the relative steering, output of the KCU model (KCU: kite control unit)
# u_d_prime: the normalized depower settings
# v_a: the apparent wind speed
# omega: the angular velocity of the kite
# implements the simulink diagram docs/kite_model.png
@with_kw mutable struct KiteModel @deftype Float64
    fcs::FPCSettings
    beta
    psi
    phi
    int_beta::Integrator = Integrator(fcs.dt, 1.0, beta) # integrator output: the elevation angle beta
    int_psi::Integrator  = Integrator(fcs.dt, 1.0, psi) # integrator output: heading angle psi, not normalized
    int_phi::Integrator  = Integrator(fcs.dt, 1.0, phi)  # integrator output: azimuth angle phi
    u_s = 0.0
    v_a = 0.0
    u_d_prime = 0.2
    k_d_s = 1.5    
    omega = 0.08
    c0 = 0.0
    c1 = 0.262
    c2 = 6.27
    a = 0.0
    m1 = 0
    psi_dot = 0
    x0 = 0
end

function KiteModel(fcs::FPCSettings; beta_0=33.0, psi_0=90.0, phi_0=0.0)
    km = KiteModel(fcs=fcs, beta=deg2rad(beta_0), psi=deg2rad(psi_0), phi=deg2rad(phi_0))
    km.m1 = km.c2 / 20.0
    km
end

function calc_x0_x1_psi_dot(km::KiteModel, x)
    x0, x1 = x[1], x[2]
    psi_dot = km.a + km.m1 * sin(x0) * cos(x1)
    x0 = calc_output(km.int_psi, psi_dot)             # self.int_psi.calcOutput(psi_dot)
    x1 = calc_output(km.int_beta, km.omega * cos(x0)) # self.int_beta.calcOutput(self.omega * cos(x0))
    x0, x1, psi_dot
end

function solve(km::KiteModel)
    function residual!(F, x)
        x0, x1, psi_dot = calc_x0_x1_psi_dot(km, x)
        F[begin]   = (x0 - x[begin]) * 0.5
        F[begin+1] = (x1 - x[begin+1])
    end
    divisor = km.u_d_prime * km.k_d_s + 1.0
    @assert abs(divisor) > EPSILON
    km.a = (km.u_s - km.c0) * km.v_a * km.c1 / divisor
    km.m1 = km.c2 / 20.0
    res = nlsolve(residual!, [ km.x0; km.beta], ftol=1e-10)
    @assert converged(res)
    x = res.zero
    x0, x1, psi_dot = calc_x0_x1_psi_dot(km, x)
    km.psi_dot = psi_dot
    km.psi = wrap2pi(x0)
    km.x0 = x0
    km.beta = x1
    km.phi = calc_output(km.int_phi, -(sin(x0) * km.omega))
end

function on_timer(km::KiteModel)
    on_timer(km.int_beta)
    on_timer(km.int_psi)
    on_timer(km.int_phi)
end
