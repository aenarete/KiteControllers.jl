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
pid1 = SpeedController(wcs)
set_v_set(pid1, -0.5)
set_tracking(pid1, -0.5)
set_inactive(pid1, false)
set_v_set_in(pid1, 4.0)
last_v_set_out = 0.0

# def force_speed_controller_test1():  
#     """

#     """      
#     # create input and output arrays
#     STARTUP = getStartUp()    
#     V_WIND = STARTUP * getTriangleWind()
#     V_RO, V_SET_OUT, FORCE, F_ERR = np.zeros(SAMPLES),  np.zeros(SAMPLES),  np.zeros(SAMPLES), np.zeros(SAMPLES) 
#     ACC, ACC_SET, V_ERR = np.zeros(SAMPLES),  np.zeros(SAMPLES),  np.zeros(SAMPLES)  
#     STATE = np.zeros((SAMPLES,), dtype=np.int)
#     # create the winch model and and the v_set_in calculator and mixer
#     winch = Winch()
#     delay = UnitDelay()
#     calc = CalcVSetIn()
#     # create and initialize speed controller 
#     pid1 = SpeedController()
#     pid1.setTracking(0.0)
#     pid1.setInactive(False)
#     pid1.setVSetIn(4.0)
#     # create and initialize lower force controller
#     pid2 = LowerForceController()
#     pid2.setFSet(F_LOW)
#     pid2.setTracking(0.0)
#     pid2.setReset(True)
#     pid2.setV_SW(-1.0) # TODO: Is this a good choice? It should not be zero, though.
#     # create the mixer for the output of the two controllers
#     mix2 = Mixer_2CH()
#     force = 0.0
#     last_force = 0.0
#     last_v_set_out = 0.0    
#     with Timer() as t1:
#         for i in range(SAMPLES):
#             # calc v_set_in of the speed controller
#             calc.setVSetPc_Force(None, last_force)
#             v_set_in = calc.getVSetIn()
#             pid1.setVSetIn(v_set_in)
#             # get the input (the wind speed)
#             v_wind = V_WIND[i]
#             v_ro = winch.getSpeed()
#             acc = winch.getAcc()
#             V_RO[i] = v_ro
#             ACC[i] = acc
#             force = calcForce(v_wind, v_ro)
#             FORCE[i] = force
#             # winch.setForce(0.5 * force + 0.5 * last_force)
#             winch.setForce(force)
#             # calculate v_set_out_A from the speed controller
#             pid1.setVAct(v_ro)            
#             v_set_out_A = pid1.getVSetOut()    
#             V_ERR[i] = pid1.getVErr()
#             # calculate v_set_out_B from the force controller
#             pid2.setForce(last_force)
#             if i * PERIOD_TIME <= STARTUP_TIME:
#                 reset = True
#             else:
#                 reset = False
#             # print "reset: ", reset
#             pid2.setReset(reset)
#             v_sw = calcV_ro(pid2._f_set) * 1.05
#             pid2.setV_SW(v_sw)
#             # pid2.setVAct(delay.calcOutput(v_ro))
#             pid2.setVAct(v_ro)
#             pid2.setTracking(v_set_out_A)
#             pid2.setForce(force)
#             v_set_out_B = pid2.getVSetOut()
#             F_ERR[i] = pid2.getFErr()
#             pid1.setTracking(v_set_out_B)
#             mix2.setInputA(v_set_out_A)
#             mix2.setInputB(v_set_out_B)
#             mix2.selectB(pid2._active) # perhaps to this later
#             pid1.setInactive(pid2._active)
#             STATE[i] = pid2._active
#             # print "reset, active", reset, pid2._active
#             v_set_out = mix2.getOutput()
#             # 
#             ACC_SET[i] = (v_set_out - last_v_set_out) / PERIOD_TIME
#             V_SET_OUT[i] = v_set_out
#             v_set = STARTUP[i] * v_set_out
#             # set the reel-out speed of the winch
#             winch.setVSet(v_set)
#             # update the state of the statefull components
#             winch.onTimer()        
#             pid1.onTimer()
#             pid2.onTimer()
#             calc.onTimer()
#             mix2.onTimer()
#             delay.onTimer()
#             # pid1.setInactive(pid2._active)
#             last_force = force
#             last_v_set_out = v_set_out            
            
#     print("time for executing the speed control loop in us: ", form((t1.secs)  / SAMPLES * 1e6))
#     return TIME, V_WIND, V_RO, V_SET_OUT, ACC, FORCE, STATE, V_ERR, F_ERR
    