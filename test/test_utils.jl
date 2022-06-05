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