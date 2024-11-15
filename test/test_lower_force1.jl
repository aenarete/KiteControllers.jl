# activate the test environment if needed
using Pkg
if ! ("ControlPlots" ∈ keys(Pkg.project().dependencies))
    using TestEnv; TestEnv.activate()
end
using KiteControllers, Timers; tic()
if false; include("../src/KiteControllers.jl"); end
if false; include("../src/winchcontroller.jl"); end
if false; include("../src/wc_components.jl"); end
if false; include("../src/wc_settings.jl"); end

# Test the lower force controller with a pre-described input for the measured force and a constant
# do NOT use the higher level WinchController
# set force of 350 N
# input: data/transition.arrow
using KiteControllers, ControlPlots, BenchmarkTools

log = load_log("transition.arrow2")
sl  = log.syslog
dt  = 0.05
t_start = 214
t_stop  = 218
force = sl.force[Int64(t_start/dt)+1:Int64(t_stop/dt)+1]
set_forces = sl.var_04[Int64(t_start/dt)+1:Int64(t_stop/dt)+1]
time  = sl.time[Int64(t_start/dt)+1:Int64(t_stop/dt)+1]
V_RO  = sl.v_reelout[Int64(t_start/dt)+1:Int64(t_stop/dt)+1]
display(plot(time, [force, set_forces], fig="forces"))
SAMPLES = length(time)
V_SET = zeros(SAMPLES)
ACC = zeros(SAMPLES)
D_F_ERR = zeros(SAMPLES)

# create and initialize winch controller
wcs = WCSettings(dt=0.02)
update(wcs)
wcs.dt = 0.05
lfc = LowerForceController(wcs)
lfc.tracking = V_RO[1]
KiteControllers._set(lfc)
lfc.v_act = V_RO[1]
lfc.force = force[1]
lfc.f_set = 350
KiteControllers._update_reset(lfc)

for i in 1:SAMPLES
    # controller
    lfc.force = force[i]
    lfc.v_act = V_RO[i]
    D_F_ERR[i] = lfc.f_err- lfc.last_err
    V_SET[i] = get_v_set_out(lfc)
    if i > 1
        ACC[i] = (V_SET[i]-V_SET[i-1])/wcs.dt
    end
    on_timer(lfc)
end
display(plot(time, ACC; xlabel="Time [s]", ylabel="ACC", fig="acc"))
display(plot(time, D_F_ERR; xlabel="Time [s]", ylabel="D_F_ERR", fig="d(f_err)"))
display(plot(time, [V_RO, V_SET]; xlabel="Time [s]", ylabel="v_ro [m/s]", labels=["V_RO", "V_SET"], fig="v_ro"))

