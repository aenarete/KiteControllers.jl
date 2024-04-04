# activate the test environment if needed
using Pkg
if ! ("ControlPlots" ∈ keys(Pkg.project().dependencies))
    using TestEnv; TestEnv.activate()
end
using Timers; tic()

# Test the speed controller. , using a reel- out speed, proportional to the sqare-root
# of the force.
# Input: A varying wind speed. Implements the simulink block diagram, shown in
# docs/speed_controller_test2.png
using KiteControllers, ControlPlots, BenchmarkTools

wcs = WCSettings()

DURATION = 10.0
SAMPLES = Int(DURATION / wcs.dt + 1)
TIME = range(0.0, DURATION, SAMPLES)
V_WIND_MAX = 8.0 # max wind speed of test wind
V_WIND_MIN = 4.0 # min wind speed of test wind
FREQ_WIND  = 0.25 # frequency of the triangle wind speed signal 
BENCHMARK = false

include("test_utils.jl")

STARTUP = get_startup(wcs)    
V_WIND = STARTUP .* get_triangle_wind(wcs)
V_RO = zeros(SAMPLES)
V_SET_OUT = zeros(SAMPLES)
FORCE = zeros(SAMPLES)
ACC = zeros(SAMPLES)
ACC_SET = zeros(SAMPLES)
winch = Winch(wcs)
calc = CalcVSetIn(wcs)
pid1 = SpeedController(wcs)
set_tracking(pid1, 0)
set_inactive(pid1, true)
set_inactive(pid1, false)
set_v_set_in(pid1, 4.0)
last_v_set_out = 0.0
last_force = Ref(0.0)
if BENCHMARK
    b = @benchmark for i in 1:SAMPLES
        speed_controller_step2!(pid1, winch, calc, i, last_force, last_v_set_out, V_WIND, STARTUP, ACC, FORCE, ACC_SET, V_SET_OUT)
    end
else
    for i in 1:SAMPLES
        speed_controller_step2!(pid1, winch, calc, i, last_force, last_v_set_out, V_WIND, STARTUP, ACC, FORCE, ACC_SET, V_SET_OUT)
    end
end

p=plotx(TIME, V_WIND, V_RO, V_SET_OUT, ACC, FORCE*0.001;
      ylabels=["v_wind [m/s]", "v_reel_out [m/s]", "v_set_out [m/s]", "acc [m/s²]", "force [kN]"],
      fig="test_speedcontroller2")
display(p)

println("Max iterations needed: $(wcs.iter)")
if BENCHMARK println("Average time per control step: $(mean(b.times)/SAMPLES/1e9) s") end
toc()
nothing
