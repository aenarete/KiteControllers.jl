# observe the flight path and collect useful information to optimize it
mutable struct KiteObserver
    time::Vector{Float64}
    length::Vector{Float64}
    fig8::Vector{Int64}
    elevation::Vector{Float64}
    corr_vec::Vector{Float64}
end
function KiteObserver()
    KiteObserver(Float64[], Float64[], Int64[], Float64[], Float64[])
end
function observe!(ob::KiteObserver, log::SysLog, elev_nom=26)
    sl  = log.syslog
    last_sign = -1
    for i in 1:length(sl.azimuth)
        # only look at the second cycle
        if sl.var_01[i] == 2 &&  sl.sys_state[i] in (6, 8)
            if sign(sl.azimuth[i]) != last_sign
                push!(ob.time, sl.time[i])
                push!(ob.length, sl.l_tether[i])
                push!(ob.fig8, sl.var_02[i])
                push!(ob.elevation, rad2deg(sl.elevation[i]))
            end
            last_sign = sign(sl.azimuth[i])
        end
    end
    elev_right = elev_nom
    elev_left = elev_nom
    for fig8 in 0:maximum(ob.fig8)
        for i in 1:length(ob.fig8)
            if ob.fig8[i]==fig8
                if isodd(i)
                    cor_right=(elev_nom-ob.elevation[i])
                    push!(ob.corr_vec, cor_right)
                else
                    cor_left=(elev_nom-ob.elevation[i])
                    push!(ob.corr_vec, cor_left)
                end
               
               
            end
        end
    end
end

# calculate the corrected elevations per figure-of-eight, one for the left and one for the right attractor point
function corrected_elev(ob, fig8, elev_nom)
    fig8=Int64(round(fig8))
    if ! isnothing(ob)
        elev_right = elev_nom + ob.corr_vec[2fig8+1]
        if 2fig8+2 <= length(ob.corr_vec)
            elev_left = elev_nom + ob.corr_vec[2fig8+2]
        else
            elev_left = elev_right
        end
    else
        elev_right=elev_nom
        elev_left=elev_nom
    end
    elev_right, elev_left
end

function residual()
    # run a simulation and save the result
    # calculate the residual
    # return a vector
end

# correction for first (lowest) turn
function corrected_elev(ob, elev_nom)
    if isnothing(ob) || length(ob.time) == 0
        return elev_nom
    end
    elev_nom + 10.13+5
end