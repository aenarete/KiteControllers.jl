# def speed_controller_test1():
#     """
#     Test 3: Test the speed controller. 
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