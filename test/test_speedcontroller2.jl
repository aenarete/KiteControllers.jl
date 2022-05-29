# def speed_controller_test2():
#     """
#     Test the speed controller, using a reel- out speed, proportional to the sqare-root
#     of the force.
#     Input: A varying wind speed. Implements the simulink block diagram, shown in
#     ./01_doc/speed_controller_test2.png
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
#     calc = CalcVSetIn()
#     pid1 = SpeedController()
#     # pid1.setVSet(0.0)
#     pid1.setTracking(0.0)
#     pid1.setInactive(True)
#     pid1.setInactive(False)
#     pid1.setVSetIn(4.0)
#     last_v_set_out = 0.0
#     force = 0.0
#     last_force = 0.0
#     with Timer() as t1:
#         for i in range(SAMPLES):
#             # calc v_set_in
#             calc.setVSetPc_Force(None, force)
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
#             winch.setForce(0.5 * force + 0.5 * last_force)
#             pid1.setVAct(v_ro)
#             v_set_out = pid1.getVSetOut()
#             # print "v_set_in, v_set_out, v_ro", v_set_in, v_set_out, v_ro, pid1._inactive
#             ACC_SET[i] = (v_set_out - last_v_set_out) / PERIOD_TIME
#             V_SET_OUT[i] = v_set_out
#             v_set = STARTUP[i] * v_set_out
#             # set the reel-out speed of the winch
#             winch.setVSet(v_set)
#             # update the state of the statefull components
#             winch.onTimer()        
#             pid1.onTimer()
#             calc.onTimer()           
#             delay.onTimer()
#             last_force = force
#             last_v_set_out = v_set_out
#     print("time for executing the speed control loop in us: ", form((t1.secs)  / SAMPLES * 1e6))
#     return TIME, V_WIND, V_RO, V_SET_OUT, ACC, FORCE  