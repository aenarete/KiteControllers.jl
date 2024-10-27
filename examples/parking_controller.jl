# prototype of a parking controller
# Components: PID controller, NDI block, and gain scheduler
using DiscretePIDs, Parameters

@with_kw mutable struct ParkingControllerSettings @deftype Float64
    dt
    # turn rate controller settings
    kp_tr=1 # can become a vector when we start to implement a parameter varying controller
    ki_tr=0
    kd_tr=0
    N_tr = 10
    # outer controller (heading/ course) settings
    kp=1
    ki=0
    kd=0
    N = 10
    # NDI block settings
    va_min = 5.0   # minimum apparent wind speed
    va_max = 100.0 # maximum apparent wind speed
    k_ds = 2.0 # influence of the depower settings on the steering sensitivity
    c1 = 0.048 # v9 kite model
    c2 = 5.5   
end

struct ParkingController
    pcs::ParkingControllerSettings
    pid_tr::DiscretePID
    pid_outer::DiscretePID
end

function ParkingController(pcs::ParkingControllerSettings)
    Ti = pcs.kp_tr/ pcs.ki_tr
    Td = pcs.kd_tr/ pcs.kp_tr
    Ts = pcs.dt
    pid_tr = DiscretePID(;K=pcs.kp_tr, Ti, Td, Ts, N=pcs.N_tr)
    pid_outer = DiscretePID(;K=pcs.kp, Ti, Td, Ts, N=pcs.N)
    return ParkingController(pcs, pid_tr, pid_outer)
end

"""
    linearize(pcs::ParkingControllerSettings, psi_dot, psi, elevation, v_app; ud_prime=0)

Nonlinear, dynamic inversion block (NDI) according to Eq. 6.4 and Eq. 6.12.

Parameters:
- psi_dot: desired turn rate in radians per second
- psi: heading in radians
- elevation: elevation angle in radians
- v_app: apparent wind speed in m/s
- ud_prime: depower setting in the range of 0 to 1, 0 means fully powered, 1 means fully depowered
"""
function linearize(pcs::ParkingControllerSettings, psi_dot, psi, elevation, v_app; ud_prime=0)
    # Eq. 6.13: calculate va_hat
    va_hat = clamp(v_app, pcs.va_min, pcs.va_max)
    # Eq. 6.12: calculate the steering from the desired turn rate
    u_s = (1.0 + pcs.k_ds * ud_prime) / (pcs.c1 * va_hat) * (psi_dot - pcs.c2 / va_hat * sin(psi) * cos(elevation))
    if abs(psi_dot) < 1e-6
        psi_dot = 1e-6
    end
    ndi_gain = clamp(u_s / psi_dot, -20.0, 20.0)
    return u_s, ndi_gain
end

function calc_steering(pc::ParkingController, heading; elevation=0.0, v_app=10.0, ud_prime=0.0)
    # # calculate the desired turn rate
    # psi_dot = update(pc.pid_outer, heading)
    # # linearize the NDI block
    # u_s, ndi_gain = linearize(pc.pcs, psi_dot, heading, elevation, v_app; ud_prime)
    # # calculate the steering
    # steering = update(pc.pid_tr, u_s)
    return steering
end

function main()
    # set the parameters of the parking controller
    pcs = ParkingControllerSettings(kp_tr=1.05, ki_tr=0.012, kd_tr=13.25*2.0, dt=0.05)
    # create the parking controller
    pc = ParkingController(pcs)
    # set the desired turn rate
    psi_dot = 0.1
    # set the heading
    psi = 0.0
    # set the elevation angle
    elevation = 0.0
    # set the apparent wind speed
    v_app = 10.0
    # set the depower setting
    ud_prime = 0.5
    # linearize the NDI block
    u_s, ndi_gain = linearize(pc.pcs, psi_dot, psi, elevation, v_app; ud_prime)
    println("u_s: $u_s, ndi_gain: $ndi_gain")
end
