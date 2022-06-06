"""
Settings of the WinchController
"""
@with_kw mutable struct WCSettings @deftype Float64
    "timestep of the winch controller"
    dt = 0.02
    fac = 0.25
    eps = 1e-6
    "max iterations of the PID solvers"
    max_iter::Int64 = 100
    iter::Int64 = 0
    "startup time for soft start"
    t_startup = 0.25
    "blending time of the mixers in seconds"
    t_blend = 0.25
    "limitation of the reel-out speed error, used by the input saturation block of the speed controller"
    v_sat_error = 1.0
    "limitation of the reel-out speed , used by the output saturation block of the speed controller"
    v_sat = 12.0 # was: 8.0
    "maximal reel-in speed [m/s]"
    v_ri_max = 12.0
    "P value of the speed controller"
    p_speed = 0.125
    "I value of the speed controller"
    i_speed = 4.0
    "back calculation constant for the anti-windup loop of the speed controller"
    kb_speed = 4.0
    "tracking constant of the speed controller"
    kt_speed = 5.0
    "reel-out velocity where the set force should reach it's maximum"
    vf_max = 2.75
    "P constant of the lower force controller"
    pf_low = 1.44e-4
    "I constant of the lower force controller"
    if_low = 7.5e-3 * 1.5
    "back calculation constant for the anti-windup loop of the lower force controller"
    kbf_low = 1.0
    "tracking constant of the lower force controller"
    ktf_low = 8.0
    "lower force limit [N]"
    f_low = 300
    "upper force limit [N]"
    f_high = 4000
    "P constant of upper force controller"
    pf_high = 0.007 * 0.33
    "I constant of upper force controller"
    if_high = 0.003 * 0.33
    "D constant of upper force controller"
    df_high = 2e-5 * 2.0 * 0.0
    "filter constant n of upper force controller"
    nf_high = 15.0
    "back calculation constant for the anti-windup loop of the upper force controller"
    kbf_high = 1.0
    "tracking constant of the upper force controller"
    ktf_high = 10.0
    "interations of the winch model"
    winch_iter = 10
    "maximal acceleration of the winch (derivative of the set value of the reel-out speed)"
    max_acc = 8.0
    "proportional factor of the square root law, see function calc_vro"
    kv = 0.06
end

function pf_low_scaled(wcs::WCSettings)
   wcs.pf_low * wcs.fac
end

function if_low_scaled(wcs::WCSettings)
   wcs.if_low * wcs.fac
end
