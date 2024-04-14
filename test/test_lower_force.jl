# activate the test environment if needed
using Pkg
if ! ("ControlPlots" ∈ keys(Pkg.project().dependencies))
    using TestEnv; TestEnv.activate()
end
using KiteControllers, KiteUtils, Timers; tic()

# Test the lower force controller with a pre-described input for the measured force and a constant
# set force of 350 N
# input: data/transition.arrow
using KiteControllers, ControlPlots, BenchmarkTools

log = load_log("transition.arrow")
sl  = log.syslog
dt  = 0.05
t_start = 216
t_stop  = 220
force = sl.force[Int64(t_start/dt)+1:Int64(t_stop/dt)+1]
set_forces = sl.var_04[Int64(t_start/dt)+1:Int64(t_stop/dt)+1]
time  = sl.time[Int64(t_start/dt)+1:Int64(t_stop/dt)+1]
V_RO  = sl.v_reelout[Int64(t_start/dt)+1:Int64(t_stop/dt)+1]
display(plot(time, [force, set_forces]))
SAMPLES = length(time)
V_SET = zeros(SAMPLES)

wcs = WCSettings()
wcs.test = true
wcs.f_low = 350
wcs.fac = 1.0
wcs.t_blend = 0.25
wcs.pf_high = 1.44e-4*1.6*0.5

# STARTUP = get_startup(wcs)    
# V_WIND = STARTUP .* get_triangle_wind(wcs)
# V_RO, V_SET_OUT, FORCE, F_ERR = zeros(SAMPLES), zeros(SAMPLES), zeros(SAMPLES), zeros(SAMPLES)
# ACC, ACC_SET, V_ERR = zeros(SAMPLES), zeros(SAMPLES), zeros(SAMPLES)
# RESET, ACTIVE, F_SET = zeros(SAMPLES), zeros(SAMPLES), zeros(SAMPLES)
# STATE = zeros(Int64, SAMPLES)
# create and initialize winch controller 
wc = WinchController(wcs)
winch = KiteControllers.Winch(wcs)
f_low = wcs.f_low

for i in 1:SAMPLES

    set_force(winch, force[i])

    # controller
    V_SET[i] = calc_v_set(wc, V_RO[i], force[i], f_low)
    
    on_timer(wc)

#     # logging
#     acc   = get_acc(winch)
#     state = get_state(wc)
#     status = get_status(wc)
#     ACC[i] = acc 
#     STATE[i] = state
#     V_RO[i] = v_act
#     # FORCE[i] = force
#     V_SET_OUT[i] = v_set
#     RESET[i] = status[1]
#     ACTIVE[i] = status[2]
#     FORCE[i] = status[3]
#     F_SET[i] = status[4]
#     V_SET_OUT[i] = status[5]
#     if state in [0,2]
#         F_ERR[i] = FORCE[i] - F_SET[i]
#     else
#         V_ERR[i] = V_RO[i] - v_set
#     end
end
display(plot(time, [V_RO, V_SET], fig="v_ro"))

