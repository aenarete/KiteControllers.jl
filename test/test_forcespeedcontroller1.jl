# activate the test environment if needed
using Pkg
if ! ("Plots" ∈ keys(Pkg.project().dependencies))
    using TestEnv; TestEnv.activate()
end
using Timers; tic()

# Test the speed controller in combination with the controller for the lower force.
# Input: A varying wind speed. Implements the simulink block diagram, shown in
# docs/force_speed_controller_test1.png
using KiteControllers, Plots, BenchmarkTools
inspectdr()

wcs = WCSettings()

DURATION = 10.0
SAMPLES = Int(DURATION / wcs.dt + 1)
TIME = range(0.0, DURATION, SAMPLES)
V_WIND_MAX = 6.0 # max wind speed of test wind
V_WIND_MIN = 0.0 # min wind speed of test wind
FREQ_WIND  = 0.25 # frequency of the triangle wind speed signal 
BENCHMARK = false

include("test_utils.jl")

STARTUP = get_startup(wcs)    
V_WIND = STARTUP .* get_triangle_wind(wcs)
V_RO, V_SET_OUT, FORCE, F_ERR = zeros(SAMPLES), zeros(SAMPLES), zeros(SAMPLES), zeros(SAMPLES)
ACC, ACC_SET, V_ERR = zeros(SAMPLES), zeros(SAMPLES), zeros(SAMPLES)
STATE = zeros(Int64, SAMPLES)
# create the winch model and and the v_set_in calculator and mixer
winch = Winch(wcs)
calc = CalcVSetIn(wcs)
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
mix2 = Mixer_2CH(wcs.dt)
last_force = Ref(0.0)
last_v_set_out = Ref(0.0)

for i in 1:SAMPLES
    speed_controller_step3!(pid1, pid2, winch, calc, i, last_force, last_v_set_out, V_WIND, STARTUP, V_RO, ACC, FORCE, V_SET_OUT, STATE, V_ERR, F_ERR)
end

p1=plot(TIME, V_WIND,    label="v_wind [m/s]",   width=2, xtickfontsize=12, ytickfontsize=12, legendfontsize=12)
plot!(TIME, V_RO,      label="v_reel_out [m/s]", width=2, xtickfontsize=12, ytickfontsize=12, legendfontsize=12)
plot!(TIME, V_SET_OUT, label="v_set_out [m/s]",  width=2, xtickfontsize=12, ytickfontsize=12, legendfontsize=12)
plot!(TIME, ACC,       label="acc [m/s²]",       width=2, xtickfontsize=12, ytickfontsize=12, legendfontsize=12)
plot!(TIME, FORCE*0.001, label="force [kN]",     width=2, xtickfontsize=12, ytickfontsize=12, legendfontsize=12)
plot!(TIME, STATE,       label="state",          width=2, xtickfontsize=12, ytickfontsize=12, legendfontsize=12)
plot!(TIME, V_ERR, label="v_error [m/s]",        width=2, xtickfontsize=12, ytickfontsize=12, legendfontsize=12)
plot!(TIME, F_ERR*0.001, label="f_error [kN]",   width=2, xtickfontsize=12, ytickfontsize=12, legendfontsize=12)

pIDR = display(p1)           # Display with InspectDR and keep plot object
resize!(pIDR.wnd, 1200, 700) # Resize GTK window directly

println("Max iterations needed: $(wcs.iter)")
            
#     return TIME, V_WIND, V_RO, V_SET_OUT, ACC, FORCE, STATE, V_ERR, F_ERR
    