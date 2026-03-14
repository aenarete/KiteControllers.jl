# FlightPathController as specified in chapter six of
# the PhD thesis of Uwe Fechner.

const TAU = 0.95   # time constant for psi_dot_set
const TAU_VA = 0.0 # time constant for averaging the apparent wind velocity

"""
FlightPathController as specified in chapter six of the PhD thesis of Uwe Fechner.

Main inputs are calls to the functions:
- `on_control_command()``
- `on_est_sys_state()`

Main output is the set value of the steering `u_s`, returned by the method:
- `calc_steering()`

Once per time step the method
- `on_timer()`
must be called.

See also:
`docs/flight_path_controller_I.png` and
`docs/flight_path_controller_II.png` and
`docs/flight_path_controller_III.png`

# Fields
- `fcs::`[`FPCSettings`](@ref): settings of the flight path controller.
- `count::Int64`: cycle counter used for periodic debug output.
- `attractor::MVector{2, Float64}`: current attractor point [azimuth, elevation] in radians.
- `psi_dot_set`: desired turn rate [rad/s], or `nothing` when navigating to an attractor.
- `psi_dot_set_final`: final desired turn rate [rad/s] after the low-pass filter settles.
- `phi`: azimuth angle of the kite [rad].
- `beta`: elevation angle of the kite [rad].
- `psi`: heading of the kite [rad].
- `chi`: course (track angle) of the kite [rad].
- `chi_factor`: blending weight between heading and course (0 = heading only, 1 = course only).
- `omega`: angular velocity of the kite on the unit sphere [°/s].
- `est_psi_dot`: estimated turn rate based on heading change [rad/s].
- `est_chi_dot`: estimated turn rate based on course change [rad/s].
- `chi_set`: desired flight direction (bearing) computed by `navigate()` [rad].
- `u_d0`: minimum depower setting (fully powered kite).
- `u_d_max`: maximum depower setting (fully depowered kite).
- `u_d_prime`: normalized depower setting in [0, 1].
- `u_s_max`: saturation limit for the steering output.
- `psi_dot_max`: saturation limit for the commanded turn rate [rad/s].
- `u_d`: actual depower setting in [0, 1].
- `k_c2`: active c₂ scaling factor for reel-out phase (copied from `fcs.k_c2`).
- `k_c2_int`: active c₂ scaling factor for intermediate phases (copied from `fcs.k_c2_int`).
- `k_c2_hight`: active c₂ scaling factor for high-elevation reel-out (copied from `fcs.k_c2_high`).
- `c1`: effective NDI coefficient c₁ = `fcs.c1 * fcs.k_c1` [rad/m].
- `c2`: effective NDI coefficient c₂ (updated each step based on apparent wind speed).
- `intermediate::Bool`: `true` while in an intermediate planner phase (`LOW_*`).
- `va`: apparent wind speed at the kite [m/s].
- `va_av`: low-pass filtered apparent wind speed [m/s].
- `va_min`: minimum apparent wind speed used for full NDI [m/s].
- `ndi_gain`: quotient of NDI output over input (used for anti-windup).
- `int::Integrator`: integrator for the I term of the PID controller.
- `int2::Integrator`: integrator for the D term of the PID controller.
- `k_u`: anti-windup gain for the saturated steering signal.
- `k_psi`: anti-windup gain for the saturated turn rate.
- `err`: heading/course error fed into the PID controller [rad].
- `res::MVector{2, Float64}`: residual vector of the nonlinear solver.
- `u_s`: steering output of the FPC in [-1, 1].
- `k_psi_out`, `k_u_out`, `k_psi_in`, `k_u_in`: internal anti-windup signals.
- `int_in`, `int2_in`: inputs to the I and D integrators (for logging).
- `reset_int1::Bool`: flag to reset the main integrator at the next `calc_steering` call.
- `radius`: commanded turn radius [m], or `nothing` when not in radius-control mode.
- `_n`: filter coefficient for the discrete derivative in the D term.
- `_i`: call counter for `calc_steering`; used to trigger one-shot initializations.
"""
@with_kw mutable struct FlightPathController @deftype Float64
    "struct holding the settings of the flight path controller"
    fcs::FPCSettings
    "cycle number"
    count::Int64                               = 0
    "attractor coordinates, azimuth and elevation in radian"
    attractor::MVector{2, Float64}             = zeros(2)
    "desired turn rate in rad per second or nothing"
    psi_dot_set::Union{Nothing, Float64}       = nothing
    psi_dot_set_final::Union{Nothing, Float64} = nothing
    "azimuth angle of the kite position in radian"
    phi                                        = 0
    "elevation angle of the kite position in radian"
    beta                                       = 0
    "heading of the kite in radian"
    psi                                        = 0
    "course in radian"
    chi                                        = 0
    "0.0 use psi only; 1.0 use chi only"
    chi_factor                                 = 0
    "angular velocity of the kite in degrees/s"
    omega                                      = 0
    "estimated turn rate (heading)"
    est_psi_dot                                = 0
    "estimated turn rate (course)"
    est_chi_dot                                = 0
    "desired flight direction (bearing)"
    chi_set                                    = 0
    "minimal value of the depower setting, needed for the fully powered kite"
    u_d0                                       # = 0.01 * se().depower_offset
    "maximal value of the depower setting, needed for the fully depowered kite"
    u_d_max                                    = 0.01 * 42.2 # TODO: add this to settings.yaml
    "normalized depower settings"
    u_d_prime                                  = 0.2
    "maximal value of the steering settings"
    u_s_max                                    = 0.99
    "maximal value of the turn rate in radians per second"
    psi_dot_max                                = 3.0
    "actual depower setting (0..1)"
    u_d                                        # = 0.01 * se().depower
    k_c2                                       = fcs.k_c2
    k_c2_int                                   = fcs.k_c2_int
    k_c2_hight                                 = fcs.k_c2_high
    c1                                         = fcs.c1 * fcs.k_c1  # identified value for hydra kite from paper [rad/m]
    c2                                         = fcs.c2 * fcs.k_c2
    intermediate::Bool                         = true
    "apparent wind speed at the kite"
    va                                         = 0.0
    "average apparent wind speed"
    va_av                                      = 0.0
    "minimal apparent wind speed for full NDI"
    va_min                                     = 8.0
    "quotient of the output and the input of the NDI block"
    ndi_gain                                   = 1.0
    "integrator for the I part of the pid controller"
    int::Integrator                            = Integrator(fcs.dt)
    "integrator for the D part of the pid controller"
    int2::Integrator                           = Integrator(fcs.dt)
    "anti-windup gain for limited steering signal"
    k_u                                        =  5.0
    "anti-windup gain for limited turn rate"
    k_psi                                      = 10.0
    "heading/ course error (input of the PID controller)"
    err                                        =  0
    "residual of the solver"
    res::MVector{2, Float64}                   = zeros(2)
    "steering output of the FPC, calculated by solve()"
    u_s                                        = 0
    k_psi_out                                  = 0
    k_u_out                                    = 0
    k_psi_in                                   = 0
    k_u_in                                     = 0
    int_in                                     = 0
    int2_in                                    = 0
    reset_int1::Bool                           = false
    radius::Union{Nothing, Float64}            = nothing
    _n                                         = 15
    "number of calls of solve"
    _i                                         = 0
end

function FlightPathController(fcs::FPCSettings; u_d0, u_d)
    FlightPathController(fcs=fcs, u_d0=u_d0, u_d=u_d)
end

"""
    on_control_command(fpc, attractor=nothing, psi_dot_set=nothing, radius=nothing, intermediate = true)

Input:  
Either the attractor point (MVector of azimuth and elevation in radian),
or `psi_dot`, the set value for the turn rate in degrees per second.
"""
function on_control_command(fpc::FlightPathController; attractor=nothing, psi_dot_set=nothing, radius=nothing, intermediate = true)
    fpc.intermediate = intermediate
    if fpc.fcs.use_radius && ! isnothing(radius)
        psi_dot_set = rad2deg(fpc.omega / radius) # desired turn rate during the turns
    end
    if ! isnothing(psi_dot_set) && ! isnothing(radius)
        temp = fpc.omega / radius
        if fpc.fcs.prn
            @printf "--->>--->> temp, psi_dot_set %.2f %.2f %.2f %.2f\n" temp psi_dot_set fpc.omega radius
        end
    end
    fpc.radius = radius
    if isnothing(psi_dot_set) && ! isnothing(fpc.psi_dot_set)
        # reset integrator
        fpc.reset_int1 = true
    end
    if ! isnothing(attractor)
        fpc.attractor .= attractor .* [-1.0, 1.0]
    end
    if ! isnothing(psi_dot_set)
        fpc.psi_dot_set_final = deg2rad(psi_dot_set)
        if !isnothing(fpc.psi_dot_set_final)
            fpc.psi_dot_set = fpc.psi_dot_set_final::Float64 * 2.0
        end
    else
        fpc.psi_dot_set = nothing
    end
end

"""
    on_est_sysstate(fpc, phi, beta, psi, chi, omega, v_a; u_d=nothing, u_d_prime=nothing)

Parameters:
- phi:      azimuth angle of the kite position in radian
- beta:     elevation angle of the kite position in radian
- psi:      heading of the kite in radian
- chi:      course of the kite in radian
- omega:    angular velocity of the kite on the unit sphere in degrees/s ???
- `u_d`:      depower settings             [0..1]
- `u_d_prime` normalized depower settings  [0..1]

Either `u_d` or `u_d_prime` must be provided.
"""
function on_est_sysstate(fpc::FlightPathController, phi, beta, psi, chi, omega, va; u_d=nothing, u_d_prime=nothing)
    fpc.phi = phi
    fpc.omega = omega
    fpc.beta = beta
    if fpc._i > 0
        delta = psi - fpc.psi
        if delta < -pi
            delta += 2π
        elseif delta > π
            delta -= 2π
        end
        fpc.est_psi_dot = delta / fpc.fcs.dt
        delta1 = chi - fpc.chi
        if delta1 < -pi
            delta1 += 2π
        elseif delta1 > π
            delta1 -= 2π
        end
        fpc.est_chi_dot = delta1 / fpc.fcs.dt
    end
    fpc.psi = psi
    fpc.chi = chi
    # Eq. 6.4: calculate the normalized depower setting
    if isnothing(u_d_prime)
        if isnothing(u_d)
            error("on_est_sysstate: either u_d or u_d_prime must be provided")
        end
        fpc.u_d_prime = (u_d::AbstractFloat - fpc.u_d0) / (fpc.u_d_max - fpc.u_d0)
    else
        fpc.u_d_prime = u_d_prime
    end
    fpc.u_d = u_d
    fpc.va = va
    fpc.va_av = TAU_VA * fpc.va_av + (1.0 - TAU_VA) * va
    # print some debug info every 2.5 seconds
    fpc.count += 1
    if fpc.count >= 50 && fpc.fcs.log_level > 2
        if fpc.fcs.prn_ndi_gain
            @printf "ndi_gain: %.2f" fpc.ndi_gain
        end
        if fpc.fcs.prn_est_psi_dot
            @printf "est_psi_dot: %.2f" rad2deg(fpc.est_psi_dot)
        end
        if fpc.fcs.prn_va
            @printf "; va, va_av: %.2f, %.2f" va fpc.va_av
        end
        @printf "; chi_set: %.2f, u_s: %.2f" rad2deg(fpc.chi_set) fpc.u_s
        println()
        fpc.count = 0
    end
end

"""
    navigate(fpc, limit=50.0)

Calculate the desired flight direction `chi_set` using great circle navigation.
Limit `delta_beta` to the value of the parameter limit (in degrees).
"""
function navigate(fpc::FlightPathController, limit=50.0)
    # navigate only if steering towards the attractor point is active
    if ! isnothing(fpc.psi_dot_set)
        return nothing
    end
    phi_set  = fpc.attractor[1]
    beta_set = fpc.attractor[2]
    r_limit = deg2rad(limit)
    if beta_set - fpc.beta > r_limit
        beta_set = fpc.beta + r_limit
    elseif beta_set - fpc.beta < -r_limit
        beta_set = fpc.beta - r_limit
    end
    y = sin(phi_set - fpc.phi) * cos(beta_set)
    x = cos(fpc.beta) * sin(beta_set) - sin(fpc.beta) * cos(beta_set) * cos(phi_set - fpc.phi)
    fpc.chi_set = atan(-y, x)
end

"""
    linearize(fpc, psi_dot; fix_va=false))

Nonlinear, dynamic inversion block (NDI) according to Eq. 6.4 and Eq. 6.12.

Parameters:
- `psi_dot`: desired turn rate in radians per second
- `fix_va`: keep va fixed for the second term of the turn rate law; was useful in some
  simulink tests.
"""
function linearize(fpc::FlightPathController, psi_dot; fix_va=false)
    # Eq. 6.13: calculate va_hat
    va_hat = fpc.va_min
    if fpc.va_av >= fpc.va_min
        va_hat = fpc.va_av
    end
    # print "v_a, v_a_hat", fpc.v_a, v_a_hat
    # Eq. 6.12: calculate the steering from the desired turn rate
    if fix_va
        va_fix = 20.0
    else
        va_fix = va_hat
    end
    if fpc.intermediate
        k = (va_fix - 22) / 3.8
        fpc.c2 = fpc.fcs.c2 * (fpc.fcs.k_c2_int + k)
    else
        k = (va_fix - 22) / 3.5 # was: 4
        fpc.c2 = fpc.fcs.c2 * (fpc.fcs.k_c2 + k)
    end
    if fpc.c2 < 2.0
        fpc.c2 = 2.0
    end
    term1 = 1.0 + fpc.fcs.k_ds * fpc.u_d_prime
    term2 = fpc.c1 * va_hat
    term3 = psi_dot::AbstractFloat - fpc.c2 / va_fix * sin(fpc.psi) * cos(fpc.beta)
    u_s = term1 /  term2 * term3
    if abs(psi_dot) < 1e-6
        psi_dot = 1e-6
    end
    fpc.ndi_gain = saturate(u_s / psi_dot, -20.0, 20.0)
    if abs(fpc.ndi_gain) < 1e-6
        fpc.ndi_gain = 1e-6
    end
    return u_s
end

"""
    calc_sat1in_sat1out_sat2in_sat2out(fpc, x)

see: `docs/flight_path_controller_II.png`

Parameters:
- x: vector of `k_u_in`, `k_psi_in` and `int2_in`
"""
function calc_sat1in_sat1out_sat2in_sat2out(fpc::FlightPathController, x)
    k_u_in   = x[1]
    k_psi_in = x[2]

    # calculate I part
    int_in = fpc.fcs.i * fpc.err + fpc.k_u * k_u_in + fpc.k_psi * k_psi_in
    int_out = calc_output(fpc.int, int_in)

    # calculate D part
    int2_in = fpc._n * (fpc.err * fpc.fcs.d - fpc.int2.last_output) / (1.0 + fpc._n * fpc.fcs.dt)
    calc_output(fpc.int2, int2_in)

    # calculate P, I, D output
    sat1_in = (fpc.fcs.p * fpc.err + int_out + int2_in) * fpc.fcs.gain

    # calculate saturated set value of the turn rate psi_dot
    sat1_out = saturate(sat1_in, -fpc.psi_dot_max, fpc.psi_dot_max)
    # nonlinear inversion
    sat2_in = linearize(fpc, sat1_out)
    # calculate the saturated set value of the steering
    sat2_out = saturate(sat2_in, -fpc.u_s_max, fpc.u_s_max)
    sat1_in, sat1_out, sat2_in, sat2_out, int_in
end

function residual_fpc!(F, x, fpc::FlightPathController)
    sat1_in, sat1_out, sat2_in, sat2_out, _ = calc_sat1in_sat1out_sat2in_sat2out(fpc, x)
    k_u_in = (sat2_out - sat2_in) / fpc.ndi_gain
    k_psi_in = sat1_out - sat1_in
    F[1] = k_u_in - x[1]
    F[2] = k_psi_in - x[2]
end

"""
    function calc_steering(fpc, parking)

Calculate the steering output `u_s` and the turn rate error `err`,
but also the signals `Kpsi_out`, `Ku_out` and `int_in`.

Implements the simulink block diagram, shown in:
`01_doc/flight_path_controller_I.png`

If the parameter parking is true, only the heading is controlled, not the course.
"""
function calc_steering(fpc::FlightPathController, parking)
    residual! = (F, x) -> residual_fpc!(F, x, fpc)

    navigate(fpc)
    # control the heading of the kite
    chi_factor = 0.0
    if fpc.omega > 0.8
            chi_factor = (fpc.omega - 0.8) / 1.2
    end
    @limit chi_factor 0.85
    fpc.chi_factor = chi_factor
    if fpc.fcs.use_chi && ! parking
        control_var = merge_angles(fpc.psi, fpc.chi, chi_factor)
    else
        fpc.chi_factor = 0.0
        control_var = fpc.psi
    end
    # println("control_var: ", control_var)
    fpc.err = wrap2pi(fpc.chi_set - control_var)
    if fpc.fcs.reset_int1 && (fpc._i == 0) || fpc.reset_int1
        if fpc.fcs.reset_int1_to_zero
            if fpc.fcs.prn
                @printf "===>>> Reset integrator to zero!\n"
            end
            reset(fpc.int::Integrator, 0.0)
        else
            if fpc.fcs.prn
                @printf "est_psi_dot: %.3f" fpc.est_psi_dot
                @printf "initial integrator output: %.3f" (fpc.est_psi_dot / fpc.fcs.gain - fpc.err * fpc.fcs.p)
            end
            reset(fpc.int::Integrator, fpc.est_psi_dot / fpc.fcs.gain - fpc.err * fpc.fcs.p)
        end
        fpc.reset_int1 = false
    end
    if fpc.fcs.reset_int2 && fpc._i == 1
        if fpc.fcs.prn
            @printf "initial output of integrator two: %.3f" fpc.err * fpc.fcs.d
        end
        reset(fpc.int2::Integrator, (fpc.err * fpc.fcs.d))
    end
    if fpc.fcs.init_opt_to_zero
        res = nlsolve(residual!, [ 0.0; 0.0], ftol=FTOL)
        @assert converged(res)
        x = res.zero
    else
        res = nlsolve(residual!, [fpc.k_u_in; fpc.k_psi_in], ftol=FTOL)
        @assert converged(res)
        x = res.zero
    end
    # println("x: $x") # x:  [-0.65423668 -5.46166397]
    sat1_in, sat1_out, sat2_in, sat2_out, int_in = calc_sat1in_sat1out_sat2in_sat2out(fpc, x)
    fpc.k_u_in = (sat2_out - sat2_in) / fpc.ndi_gain
    fpc.k_psi_in = sat1_out - sat1_in
    fpc.k_psi_out = (sat1_out - sat1_in) * fpc.k_psi
    fpc.k_u_out = (sat2_out - sat2_in) * fpc.k_u
    # @printf "sat1_in, sat1_out, sat2_in, sat2_out: %.3f, %.3f, %.3f, %.3f", sat1_in, sat1_out, sat2_in, sat2_out
    fpc.int_in = int_in
    if ! isnothing(fpc.psi_dot_set)
        if fpc.fcs.use_radius && ! isnothing(fpc.radius)
            fpc.psi_dot_set_final = fpc.omega / fpc.radius::Float64 # desired turn rate during the turns
        end
        fpc.psi_dot_set = fpc.psi_dot_set::Float64 * TAU + fpc.psi_dot_set_final::Float64 * (1-TAU)
        fpc.u_s = saturate(linearize(fpc, fpc.psi_dot_set), -1.0, 1.0)
    else
        fpc.u_s = sat2_out
    end
    fpc._i += 1
    fpc.u_s
end

function on_timer(fpc::FlightPathController)
    on_timer(fpc.int::Integrator)
    on_timer(fpc.int2::Integrator)
end

function get_state(fpc::FlightPathController)
    if ! isnothing(fpc.psi_dot_set)
        turning = true
        value = fpc.psi_dot_set
    else
        turning = false
        value = fpc.attractor
    end
    turning, value
end
