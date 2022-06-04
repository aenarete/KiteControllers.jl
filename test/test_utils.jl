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