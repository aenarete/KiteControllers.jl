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
wcs.pf_high = 1.44e-4*1.6*0.5
# wcs.df_high *= 0.5

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
v_act = 0.0
for i in 1:SAMPLES
    global v_act
    # model
    v_wind = V_WIND[i]
    force = calc_force(v_wind, v_act)
    set_force(winch, force)
    v_act = get_speed(winch)

    # controller
    v_set = calc_v_set(wc, v_act, force, f_low)
    on_timer(wc)

    # update model
    set_v_set(winch, v_set)
    on_timer(winch)

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

p1=plot(TIME, V_WIND,    label="v_wind [m/s]",   width=2, xtickfontsize=12, ytickfontsize=12, legendfontsize=12)
plot!(TIME, V_RO,      label="v_reel_out [m/s]", width=2, xtickfontsize=12, ytickfontsize=12, legendfontsize=12)
plot!(TIME, V_SET_OUT, label="v_set_out [m/s]",  width=2, xtickfontsize=12, ytickfontsize=12, legendfontsize=12)

# p2=plot(TIME, F_ERR*0.001, label="f_err [kN]",     width=2, xtickfontsize=12, ytickfontsize=12, legendfontsize=12)
# plot!(TIME, V_ERR, label="v_error [m/s]",        width=2, xtickfontsize=12, ytickfontsize=12, legendfontsize=12)

p3=#plot(TIME, ACC,       label="acc [m/s²]",       width=2, xtickfontsize=12, ytickfontsize=12, legendfontsize=12)
plot(TIME, FORCE*0.001, label="force [kN]",     width=2, xtickfontsize=12, ytickfontsize=12, legendfontsize=12)
plot!(TIME, STATE,       label="state",          width=2, xtickfontsize=12, ytickfontsize=12, legendfontsize=12)
# plot!(TIME, V_ERR, label="v_error [m/s]",        width=2, xtickfontsize=12, ytickfontsize=12, legendfontsize=12)
# plot!(TIME, F_ERR*0.001, label="f_error [kN]",   width=2, xtickfontsize=12, ytickfontsize=12, legendfontsize=12)

pIDR = display(p1)           # Display with InspectDR and keep plot object
resize!(pIDR.wnd, 1200, 700) # Resize GTK window directly

# pIDR2 = display(p2)           # Display with InspectDR and keep plot object
# resize!(pIDR2.wnd, 1200, 700) # Resize GTK window directly

pIDR3 = display(p3)
resize!(pIDR3.wnd, 1200, 700)
toc()

println("Max iterations needed: $(wcs.iter)")
