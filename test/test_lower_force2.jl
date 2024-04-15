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
display(plot(time, [force, set_forces], fig="forces"))
SAMPLES = length(time)
V_SET = zeros(SAMPLES)

wcs = WCSettings()
wcs.test = true
wcs.f_low = 350
wcs.fac = 1.0
wcs.t_blend = 0.25
wcs.pf_high = 1.44e-4*1.6*0.5

# create and initialize winch controller 
wc = WinchController(wcs)
wc.v_set_out = V_RO[1]
wc.v_set = V_RO[1]
winch = KiteControllers.Winch(wcs)
set_v_set(winch, V_RO[1])
set_v_act(wc.pid2, V_RO[1])
f_low = wcs.f_low
set_inactive(wc.pid1, true)
wc.pid2.active=true
wc.pid3.active=false
wc.pid2.v_act = V_RO[1]
wc.pid2.active = true
wc.wcs.t_startup=0.1

for i in 1:SAMPLES

    set_force(winch, force[i])
    set_v_set(winch, V_RO[i])
    set_inactive(wc.pid1, true)
    wc.pid2.active=true
    wc.pid3.active=false

    # controller
    V_SET[i] = calc_v_set(wc, V_RO[i], force[i], f_low)
    V_SET[i] = get_v_set_out(wc.pid2)
    
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

