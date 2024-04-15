# activate the test environment if needed
using Pkg
if ! ("ControlPlots" ∈ keys(Pkg.project().dependencies))
    using TestEnv; TestEnv.activate()
end
using KiteControllers, KiteUtils, Timers; tic()
if false; include("../src/KiteControllers.jl"); end
if false; include("../src/winchcontroller.jl"); end
if false; include("../src/wc_components.jl"); end
if false; include("../src/wc_settings.jl"); end

# Test the lower force controller with a pre-described input for the measured force and a constant
# do NOT use the higher level WinchController
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

# create and initialize winch controller
wcs = WCSettings()
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
    V_SET[i] = get_v_set_out(lfc)
    on_timer(lfc)
end
display(plot(time, [V_RO, V_SET], fig="v_ro"))

