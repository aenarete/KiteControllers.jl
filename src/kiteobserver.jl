# observe the flight path and collect useful information to optimize it
mutable struct KiteObserver
    time::Vector{Float64}
    length::Vector{Float64}
    fig8::Vector{Int64}
    elevation::Vector{Float64}
end
function KiteObserver()
    KiteObserver(Float64[], Float64[], Int64[], Float64[])
end
function observe!(observer::KiteObserver, log::SysLog)
    sl  = log.syslog
    last_sign = -1
    for i in 1:length(sl.azimuth)
        # only look at the second cycle
        if sl.var_01[i] == 2 &&  sl.sys_state[i] in (6, 8)
            if sign(sl.azimuth[i]) != last_sign
                push!(observer.time, sl.time[i])
                push!(observer.length, sl.l_tether[i])
                push!(observer.fig8, sl.var_02[i])
                push!(observer.elevation, rad2deg(sl.elevation[i]))
            end
            last_sign = sign(sl.azimuth[i])
        end
    end
end

# calculate the corrected elevations per figure-of-eight, one for the left and one for the right attractor point
function corrected_elev(ob, fig8, elev_nom)
    if ! isnothing(ob)
        elev_right = elev_nom
        elev_left = elev_nom
        for i in 1:length(ob.fig8)
            if ob.fig8[i]==fig8
                if isodd(i)
                    elev_right=elev_nom-(ob.elevation[i]-elev_nom)
                else
                    elev_left=elev_nom-(ob.elevation[i]-elev_nom)
                end
            end
        end
    
    else
        elev_right=elev_nom
        elev_left=elev_nom
    end
    elev_right, elev_left
end

# correction for first (lowest) turn
function corrected_elev(ob, elev_nom)
    elev_nom + 10.13+5
end