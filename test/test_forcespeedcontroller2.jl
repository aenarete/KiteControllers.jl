# activate the test environment if needed
using Pkg
if ! ("Plots" ∈ keys(Pkg.project().dependencies))
    using TestEnv; TestEnv.activate()
end
using Timers; tic()

# Test the speed controller in combination with the controller for the lower and upper force.
# Input: A varying wind speed. Implements the simulink block diagram, shown in
# docs/force_speed_controller_test2.png
using KiteControllers, Plots, BenchmarkTools
inspectdr()
InspectDR.defaults.xaxiscontrol_visible = false

wcs = WCSettings()
wcs.test = true
wcs.f_low = 350
wcs.fac = 1.0
wcs.t_blend = 0.25
#wcs.pf_low = 1.44e-4*0.5
wcs.pf_high = 1.44e-4*1.6*0.5
# wcs.kt_speed = 10


DURATION = 10.0
SAMPLES = Int(DURATION / wcs.dt + 1)
TIME = range(0.0, DURATION, SAMPLES)
V_WIND_MAX = 9.0 # max wind speed of test wind
V_WIND_MIN = 0.0 # min wind speed of test wind
FREQ_WIND  = 0.25 # frequency of the triangle wind speed signal 
BENCHMARK = false

include("test_utils.jl")

STARTUP = get_startup(wcs)    
V_WIND = STARTUP .* get_triangle_wind(wcs)
V_RO, V_SET_OUT, FORCE, F_ERR = zeros(SAMPLES), zeros(SAMPLES), zeros(SAMPLES), zeros(SAMPLES)
ACC, ACC_SET, V_ERR = zeros(SAMPLES), zeros(SAMPLES), zeros(SAMPLES)
STATE = zeros(Int64, SAMPLES)
# create and initialize speed controller 
pid1 = SpeedController(wcs)
set_tracking(pid1, 0.0)
set_inactive(pid1, false)
set_v_set_in(pid1, 4.0)
# set_v_set(pid1, -0.5)
# create and initialize lower force controller
pid2 = LowerForceController(wcs)
set_f_set(pid2, wcs.f_low)
set_tracking(pid2, 0.0)
set_reset(pid2, true)
set_v_sw(pid2, -1.0)
# create the mixer for the output of the two controllers
mix2 = Mixer_2CH(wcs.dt, wcs.t_blend)
# create winch model and unit delay and the v_set_in calculator and mixer
winch = Winch(wcs)
delay = UnitDelay()
calc = CalcVSetIn(wcs)
# create and initialize upper force controller
pid3 = UpperForceController(wcs) 
set_v_sw(pid3, calc_vro(wcs, pid3.f_set))
set_reset(pid3, true)
set_reset(pid3, false)
# create the mixer for the output of the two controllers
mix3 = Mixer_3CH(wcs.dt, wcs.t_blend)
last_force = Ref(0.0)
last_v_set_out = Ref(0.0)

for i in 1:SAMPLES
    speed_controller_step4!(pid1, pid2, pid3, mix3, winch, calc, i, last_force, last_v_set_out, V_WIND, STARTUP, V_RO, ACC, FORCE, V_SET_OUT, STATE, V_ERR, F_ERR)
end

p1=plot(TIME, V_WIND,    label="v_wind [m/s]",   width=2, xtickfontsize=12, ytickfontsize=12, legendfontsize=12)
plot!(TIME, V_RO,      label="v_reel_out [m/s]", width=2, xtickfontsize=12, ytickfontsize=12, legendfontsize=12)
plot!(TIME, V_SET_OUT, label="v_set_out [m/s]",  width=2, xtickfontsize=12, ytickfontsize=12, legendfontsize=12)

p2=plot(TIME, F_ERR*0.001, label="f_err [kN]",     width=2, xtickfontsize=12, ytickfontsize=12, legendfontsize=12)
plot!(TIME, V_ERR, label="v_error [m/s]",        width=2, xtickfontsize=12, ytickfontsize=12, legendfontsize=12)

p3=#plot(TIME, ACC,       label="acc [m/s²]",       width=2, xtickfontsize=12, ytickfontsize=12, legendfontsize=12)
plot(TIME, FORCE*0.001, label="force [kN]",     width=2, xtickfontsize=12, ytickfontsize=12, legendfontsize=12)
# plot!(TIME, STATE,       label="state",          width=2, xtickfontsize=12, ytickfontsize=12, legendfontsize=12)
# plot!(TIME, V_ERR, label="v_error [m/s]",        width=2, xtickfontsize=12, ytickfontsize=12, legendfontsize=12)
# plot!(TIME, F_ERR*0.001, label="f_error [kN]",   width=2, xtickfontsize=12, ytickfontsize=12, legendfontsize=12)

pIDR = display(p1)           # Display with InspectDR and keep plot object
resize!(pIDR.wnd, 1200, 700) # Resize GTK window directly

pIDR2 = display(p2)           # Display with InspectDR and keep plot object
resize!(pIDR2.wnd, 1200, 700) # Resize GTK window directly

pIDR3 = display(p3)
resize!(pIDR3.wnd, 1200, 700)
toc()

println("Max iterations needed: $(wcs.iter)")
            
#     return TIME, V_WIND, V_RO, V_SET_OUT, ACC, FORCE, STATE, V_ERR, F_ERR
    