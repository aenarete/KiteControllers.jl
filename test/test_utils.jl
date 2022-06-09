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
function  get_triangle_wind(wcs::WCSettings)
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

# Calculate the pulling force of the kite as function of the reel-out speed and the wind speed in the
# direction of the tether at the height of the kite. Most simplified model, massless, constant L/D,
# constant elevation angle.
function calc_force(v_wind, v_ro)
    (v_wind - v_ro)^2 * 4000.0 / 16.0
end

function speed_controller_step!(pid, winch, i, last_v_set_out, V_WIND, STARTUP, ACC, FORCE, ACC_SET, V_SET_OUT)
    # get the input (the wind speed)
    v_wind = V_WIND[i]
    v_ro = get_speed(winch)
    acc = get_acc(winch)
    V_RO[i] = v_ro
    ACC[i] = acc
    force = calc_force(v_wind, v_ro)
    FORCE[i] = force
    set_force(winch, force)
    set_v_act(pid, v_ro)
    v_set_out = get_v_set_out(pid)
    ACC_SET[i] = (v_set_out - last_v_set_out) / pid.wcs.dt
    V_SET_OUT[i] = v_set_out
    v_set = STARTUP[i] * v_set_out
    # set the reel-out speed of the winch
    set_v_set(winch, v_set)
    # update the state of the statefull components
    on_timer(winch)
    on_timer(pid)
    last_v_set_out = v_set_out
end

function speed_controller_step2!(pid, winch, calc, i, last_force, last_v_set_out, V_WIND, STARTUP, ACC, FORCE, ACC_SET, V_SET_OUT)
    # calc v_set_in
    set_vset_pc(calc, nothing, last_force[])
    v_set_in = calc_output(calc)
    set_v_set_in(pid, v_set_in)
    # get the input (the wind speed)
    v_wind = V_WIND[i]
    v_ro = get_speed(winch)
    acc = get_acc(winch)
    V_RO[i] = v_ro
    ACC[i] = acc
    force = calc_force(v_wind, v_ro)
    FORCE[i] = force
    set_force(winch, 0.5 * force + 0.5 * last_force[])
    set_v_act(pid, v_ro)
    v_set_out = get_v_set_out(pid)
    ACC_SET[i] = (v_set_out - last_v_set_out) / pid.wcs.dt
    V_SET_OUT[i] = v_set_out
    v_set = STARTUP[i] * v_set_out
    # set the reel-out speed of the winch
    set_v_set(winch, v_set)
    # update the state of the statefull components
    on_timer(winch)
    on_timer(calc)
    on_timer(pid)
    last_force[] = force
    last_v_set_out = v_set_out
end

function speed_controller_step3!(pid1, pid2, winch, calc, i, last_force, last_v_set_out, V_WIND, STARTUP, V_RO, ACC, FORCE, V_SET_OUT, V_SET_OUT_B, STATE, V_ERR, F_ERR)

    # calc v_set_in of the speed controller
    set_vset_pc(calc, nothing, last_force[])
    v_set_in = calc_output(calc)
    set_v_set_in(pid1, v_set_in)
    # get the input (the wind speed)
    v_wind = V_WIND[i]
    v_ro = get_speed(winch)
    acc = get_acc(winch)
    V_RO[i] = v_ro
    ACC[i] = acc
    force = calc_force(v_wind, v_ro)
    FORCE[i] = force
    set_force(winch, force)
    # calculate v_set_out_A from the speed controller
    set_v_act(pid1, v_ro)
    v_set_out_A = get_v_set_out(pid1)
    V_ERR[i] = get_v_error(pid1)
    # calculate v_set_out_B from the force controller
    set_force(pid2, last_force[])
    if i * wcs.dt <= wcs.t_startup
        reset = true
    else
        reset = false
    end
    set_reset(pid2, reset)
    v_sw = calc_vro(wcs, pid2.f_set) #* 1.05
    set_v_sw(pid2, v_sw)
    set_v_act(pid2, v_ro)
    set_tracking(pid2, v_set_out_A)
    set_force(pid2, force)
    v_set_out_B = get_v_set_out(pid2)
    F_ERR[i] = get_f_err(pid2)
    set_tracking(pid1, v_set_out_B)
    select_b(mix2, pid2.active)
    set_inactive(pid1, pid2.active)
    STATE[i] = pid2.active
    V_SET_OUT_B[i] = v_set_out_B
    v_set_out = calc_output(mix2, v_set_out_A, v_set_out_B)
    V_SET_OUT[i] = v_set_out
    v_set = STARTUP[i] * v_set_out
    # set the reel-out speed of the winch
    set_v_set(winch, v_set)
    # update the state of the statefull components
    on_timer(winch)
    on_timer(calc)
    on_timer(pid1)
    on_timer(pid2)
    on_timer(mix2)
    last_force[] = force
    last_v_set_out[] = v_set_out            
end

function speed_controller_step4!(pid1, pid2, winch, calc, i, last_force, last_v_set_out, V_WIND, STARTUP, V_RO, ACC, FORCE, V_SET_OUT, V_SET_OUT_B, STATE, V_ERR, F_ERR)

    # calc v_set_in of the speed controller
    set_vset_pc(calc, nothing, last_force[])
    v_set_in = calc_output(calc)
    set_v_set_in(pid1, v_set_in)
    # get the input (the wind speed)
    v_wind = V_WIND[i]
    v_ro = get_speed(winch)
    acc = get_acc(winch)
    V_RO[i] = v_ro
    ACC[i] = acc
    force = calc_force(v_wind, v_ro)
    FORCE[i] = force
    set_force(winch, force)
    # calculate v_set_out_A from the speed controller
    set_v_act(pid1, v_ro)
    v_set_out_A = get_v_set_out(pid1)
    V_ERR[i] = get_v_error(pid1)
    # calculate v_set_out_B from the force controller
    set_force(pid2, last_force[])
    if i * wcs.dt <= wcs.t_startup
        reset = true
    else
        reset = false
    end
    set_reset(pid2, reset)
    v_sw = calc_vro(wcs, pid2.f_set) #* 1.05
    set_v_sw(pid2, v_sw)
    set_v_act(pid2, v_ro)
    set_tracking(pid2, v_set_out_A)
    set_force(pid2, force)
    v_set_out_B = get_v_set_out(pid2)
    F_ERR[i] = get_f_err(pid2)
    set_tracking(pid1, v_set_out_B)
    select_b(mix2, pid2.active)
    set_inactive(pid1, pid2.active)
    STATE[i] = pid2.active
    V_SET_OUT_B[i] = v_set_out_B
    v_set_out = calc_output(mix2, v_set_out_A, v_set_out_B)
    V_SET_OUT[i] = v_set_out
    v_set = STARTUP[i] * v_set_out
    # set the reel-out speed of the winch
    set_v_set(winch, v_set)
    # update the state of the statefull components
    on_timer(winch)
    on_timer(calc)
    on_timer(pid1)
    on_timer(pid2)
    on_timer(mix2)
    last_force[] = force
    last_v_set_out[] = v_set_out            
end


    # with Timer() as t1:
    #     for i in range(SAMPLES):
    #         # calc v_set_in of the speed controller
    #         calc.setVSetPc_Force(None, last_force)
    #         v_set_in = calc.getVSetIn()
    #         pid1.setVSetIn(v_set_in)
    #         # get the input (the wind speed)
    #         v_wind = V_WIND[i]
    #         v_ro = winch.getSpeed()
    #         acc = winch.getAcc()
    #         V_RO[i] = v_ro
    #         ACC[i] = acc
    #         force = calcForce(v_wind, v_ro)
    #         FORCE[i] = force
    #         # winch.setForce(0.5 * force + 0.5 * last_force)
    #         winch.setForce(force)
    #         pid1.setVAct(v_ro)             
    #         V_ERR[i] = pid1.getVErr()
    #         pid2.setForce(last_force)
    #         if i * PERIOD_TIME <= STARTUP_TIME:
    #             reset = True
    #         else:
    #             reset = False
    #         # print "reset: ", reset
    #         pid2.setReset(reset)
    #         # set v_switch (the speed, when the force controllers shall be turned off)
    #         pid2.setV_SW(calcV_ro(pid2._f_set, F_UPPER, F_LOW) * 1.05)
    #         pid3.setV_SW(calcV_ro(pid3._f_set, F_UPPER, F_LOW) * 0.95)
    #         # pid2.setVAct(delay.calcOutput(v_ro))
    #         pid2.setVAct(v_ro)
    #         pid3.setVAct(v_ro)
    #         pid1.setTracking(mix3.getDirectOutput())
    #         pid2.setTracking(mix3.getDirectOutput())
    #         pid3.setTracking(mix3.getDirectOutput())            
    #         pid2.setForce(force)
    #         pid3.setForce(force)
    #         F_ERR[i] = pid2.getFErr() + pid3.getFErr()
            
    #         mix3.setInputA(pid1.getVSetOut()    ) # from speed controller
    #         mix3.setInputB(pid2.getVSetOut())     # from lower force controller
    #         mix3.setInputC(pid3.getVSetOut())     # from lower force controller
    #         mix3.selectB(pid2._active)  # 
    #         mix3.selectC(pid3._active)  # 
    #         pid1.setInactive((pid2._active or pid3._active))
    #         STATE[i] = mix3.getControllerState()
    #         # print "reset, active", reset, pid2._active
    #         v_set_out = mix3.getOutput()
    #         # 
    #         ACC_SET[i] = (v_set_out - last_v_set_out) / PERIOD_TIME
    #         V_SET_OUT[i] = v_set_out
    #         v_set = STARTUP[i] * v_set_out
    #         # set the reel-out speed of the winch
    #         winch.setVSet(v_set)
    #         # update the state of the statefull components
    #         winch.onTimer()        
    #         pid1.onTimer()
    #         pid2.onTimer()
    #         pid3.onTimer()
    #         calc.onTimer()
    #         mix3.onTimer()
    #         delay.onTimer()
    #         # pid1.setInactive(pid2._active)
    #         last_force = force
    #         last_v_set_out = v_set_out            
            
    # print("time for executing the speed control loop in us: ", form((t1.secs)  / SAMPLES * 1e6))
    # return TIME, V_WIND, V_RO, V_SET_OUT, ACC, FORCE, STATE, V_ERR, F_ERR 