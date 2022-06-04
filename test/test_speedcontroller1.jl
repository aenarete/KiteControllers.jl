# activate the test environment if needed
using Pkg
if ! ("Plots" ∈ keys(Pkg.project().dependencies))
    using TestEnv; TestEnv.activate()
end

using KiteControllers, Plots

wcs = WCSettings()
DURATION = 10.0
SAMPLES = Int(DURATION / wcs.dt + 1)
TIME = range(0.0, DURATION, SAMPLES)
V_WIND_MAX = 8.0 # max wind speed of test wind
V_WIND_MIN = 4.0 # min wind speed of test wind
FREQ_WIND  = 0.25 # frequency of the triangle wind speed signal 

# Create a signal, that is rising with wcs.t_startup from zero to one and then stays constant.
function get_startup(wcs::WCSettings)
    result = zeros(SAMPLES)
    startup = 0.0
    rising = true
    delta = wcs.dt/wcs.t_startup
    for i in 1:SAMPLES
        result[i] = startup
        if rising
            startup += delta
        end
        if startup >= 1.0
            startup = 1.0
            rising = false
        end
    end
    result
end

# Create a wind signal in triangle form, using the constants V_WIND_MIN and V_WIND_MAX with
# the frequency FREQ_WIND.
function  get_triangle_wind(wcs)
    result = zeros(SAMPLES)
    v_wind = 0.0
    rising = true
    delta = FREQ_WIND * 2.0 * (V_WIND_MAX - V_WIND_MIN) * wcs.dt
    for i in 1:SAMPLES
        result[i] = v_wind
        if rising
            v_wind += delta
        end
        if v_wind >= V_WIND_MAX
            v_wind = V_WIND_MAX
            rising = false
        end
        if ! rising
            v_wind -= delta
        end
        if v_wind <= V_WIND_MIN
            v_wind = V_WIND_MIN + delta
            rising = true
        end
    end
    result
end

# def speed_controller_test1():
#     """
#     Test the speed controller. 
#     Input: A varying wind speed. Implements the simulink block diagram, shown in
#     ./01_doc/speed_controller_test1.png
#     """    
#     STARTUP = getStartUp()    
#     V_WIND = STARTUP * getTriangleWind()
#     V_RO = np.zeros(SAMPLES)
#     V_SET_OUT = np.zeros(SAMPLES)
#     FORCE = np.zeros(SAMPLES)
#     ACC = np.zeros(SAMPLES)
#     ACC_SET = np.zeros(SAMPLES)
#     winch = Winch()
#     delay = UnitDelay()
#     pid1 = SpeedController()
#     pid1.setVSet(-0.5)
#     pid1.setTracking(-0.5)
#     pid1.setInactive(False)
#     pid1.setVSetIn(4.0)
#     last_v_set_out = 0.0
#     with Timer() as t1:
#         for i in range(SAMPLES):
#             # get the input (the wind speed)
#             v_wind = V_WIND[i]
#             v_ro = winch.getSpeed()
#             acc = winch.getAcc()
#             V_RO[i] = v_ro
#             ACC[i] = acc
#             force = calcForce(v_wind, v_ro)
#             FORCE[i] = force
#             winch.setForce(force)
#             pid1.setVAct(v_ro)    # OK
#             v_set_out = pid1.getVSetOut()
#             ACC_SET[i] = (v_set_out - last_v_set_out) / PERIOD_TIME
#             V_SET_OUT[i] = v_set_out
#             v_set = STARTUP[i] * v_set_out
#             # set the reel-out speed of the winch
#             winch.setVSet(v_set)
#             # update the state of the statefull components
#             winch.onTimer()        
#             pid1.onTimer()
#             delay.onTimer()
#             last_v_set_out = v_set_out
#     print("time for executing the speed control loop in us: ", form((t1.secs)  / SAMPLES * 1e6))
#     return TIME, V_WIND, V_RO, V_SET_OUT, ACC, FORCE

wind = get_triangle_wind(wcs)
plot(TIME, wind, width=2, ylabel="v_wind [m/s]", xtickfontsize=12, ytickfontsize=12, legendfontsize=12, size=(640,480), legend=false)