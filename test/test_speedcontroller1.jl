# activate the test environment if needed
using Pkg
if ! ("Plots" ∈ keys(Pkg.project().dependencies))
    using TestEnv; TestEnv.activate()
end

# Test the speed controller. 
# Input: A varying wind speed. Implements the simulink block diagram, shown in
# docs/speed_controller_test1.png
using KiteControllers, Plots
pyplot()

wcs = WCSettings()
# wm =  AsyncMachine()
DURATION = 10.0
SAMPLES = Int(DURATION / wcs.dt + 1)
TIME = range(0.0, DURATION, SAMPLES)
V_WIND_MAX = 8.0 # max wind speed of test wind
V_WIND_MIN = 4.0 # min wind speed of test wind
FREQ_WIND  = 0.25 # frequency of the triangle wind speed signal 

include("test_utils.jl")

STARTUP = get_startup(wcs)    
V_WIND = STARTUP .* get_triangle_wind(wcs)
V_RO = zeros(SAMPLES)
V_SET_OUT = zeros(SAMPLES)
FORCE = zeros(SAMPLES)
ACC = zeros(SAMPLES)
ACC_SET = zeros(SAMPLES)
winch = Winch()
delay = UnitDelay()
pid1 = SpeedController()
set_v_set(pid1, -0.5)
set_tracking(pid1, -0.5)
set_inactive(pid1, false)
set_v_set_in(pid1, 4.0)
last_v_set_out = 0.0
for i in 1:SAMPLES
    global last_v_set_out
    # get the input (the wind speed)
    v_wind = V_WIND[i]
    v_ro = get_speed(winch)
    acc = get_acc(winch)
    V_RO[i] = v_ro
    ACC[i] = acc
    force = calc_force(v_wind, v_ro)
    FORCE[i] = force
    set_force(winch, force)
    set_v_act(pid1, v_ro)
    v_set_out = get_v_set_out(pid1)
    ACC_SET[i] = (v_set_out - last_v_set_out) / wcs.dt
    V_SET_OUT[i] = v_set_out
    v_set = STARTUP[i] * v_set_out
    # set the reel-out speed of the winch
    set_v_set(winch, v_set)
    # update the state of the statefull components
    on_timer(winch)
    on_timer(pid1)
    on_timer(delay)
    last_v_set_out = v_set_out
end
#     return TIME, V_WIND, V_RO, V_SET_OUT, ACC, FORCE

p1=plot(TIME, V_WIND,    ylabel="v_wind [m/s]",     xtickfontsize=12, ytickfontsize=12, legendfontsize=12, size=(640,480), legend=false)
p2=plot(TIME, V_RO,      ylabel="v_reel_out [m/s]", xtickfontsize=12, ytickfontsize=12, legendfontsize=12, size=(640,480), legend=false, reuse=false)
p3=plot(TIME, V_SET_OUT, ylabel="v_set_out [m/s]",  xtickfontsize=12, ytickfontsize=12, legendfontsize=12, size=(640,480), legend=false, reuse=false)
p4=plot(TIME, ACC,       ylabel="acc [m/s²]",       xtickfontsize=12, ytickfontsize=12, legendfontsize=12, size=(640,480), legend=false, reuse=false)
display(p1); display(p2); display(p3); display(p4)