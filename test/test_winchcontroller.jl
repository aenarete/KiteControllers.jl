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

wcs = WCSettings()
wcs.test = true
wcs.f_low = 350
wcs.fac = 1.0
wcs.t_blend = 0.25
wcs.pf_high = 1.44e-4*1.6*0.5

DURATION = 10.0
SAMPLES = Int(DURATION / wcs.dt + 1)
TIME = range(0.0, DURATION, SAMPLES)
V_WIND_MAX = 9.0 # max wind speed of test wind
V_WIND_MIN = 0.0 # min wind speed of test wind
FREQ_WIND  = 0.25 # frequency of the triangle wind speed signal 

include("test_utils.jl")

STARTUP = get_startup(wcs)    
V_WIND = STARTUP .* get_triangle_wind(wcs)
V_RO, V_SET_OUT, FORCE, F_ERR = zeros(SAMPLES), zeros(SAMPLES), zeros(SAMPLES), zeros(SAMPLES)
ACC, ACC_SET, V_ERR = zeros(SAMPLES), zeros(SAMPLES), zeros(SAMPLES)
RESET, ACTIVE, F_SET = zeros(SAMPLES), zeros(SAMPLES), zeros(SAMPLES)
STATE = zeros(Int64, SAMPLES)
# create and initialize winch controller 
wc = WinchController(wcs)
winch = KiteControllers.Winch(wcs)
f_low = wcs.f_low

for i in 1:SAMPLES
    # model
    v_wind = V_WIND[i]

    v_act = get_speed(winch)
    force = calc_force(v_wind, v_act)
    set_force(winch, force)

    # controller
    v_set = calc_v_set(wc, v_act, force, f_low)
    
    # update model
    set_v_set(winch, v_set)
    
    on_timer(winch)
    on_timer(wc)

    # logging
    acc   = get_acc(winch)
    state = get_state(wc)
    status = get_status(wc)
    ACC[i] = acc 
    STATE[i] = state
    V_RO[i] = v_act
    # FORCE[i] = force
    V_SET_OUT[i] = v_set
    RESET[i] = status[1]
    ACTIVE[i] = status[2]
    FORCE[i] = status[3]
    F_SET[i] = status[4]
    V_SET_OUT[i] = status[5]
end

include("plot.jl")
plotx(TIME, V_WIND, V_RO, V_SET_OUT;
      labels=["v_wind [m/s]", "v_reel_out [m/s]", "v_set_out [m/s]"],
      fig="test_winchcontroller_a")

plotx(TIME, F_ERR*0.001, V_ERR;
      labels=["f_err [kN]","v_error [m/s]"],
      fig="test_winchcontroller_b")

plotx(TIME, FORCE*0.001, STATE;
      labels=["force [kN]","state"],
      fig="test_winchcontroller_c")

# plot!(TIME, V_ERR, label="v_error [m/s]")
# plot!(TIME, F_ERR*0.001, label="f_error [kN]")

toc()

println("Max iterations needed: $(wcs.iter)")
