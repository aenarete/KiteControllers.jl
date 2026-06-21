# Core plot functions shared by plots.jl (interactive GUI) and batch_plot.jl (CLI).
# Each function takes a log object and accepts keyword arguments for the few
# parameters that differ between the two UIs (ysize, energy source, dt).

include("plot_helpers.jl")

# ---------------------------------------------------------------------------
# Helper: find the first index where sl.cycle == n
# ---------------------------------------------------------------------------
function _cycle_index(sl, n)
    for i in 1:length(sl.cycle)
        if sl.cycle[i] == n; return i; end
    end
    return 1
end

# ---------------------------------------------------------------------------
# _plot_main
# ---------------------------------------------------------------------------
function _plot_main(log; ysize=10)
    sl = log.syslog
    display(plotx(sl.time, log.z, rad2deg.(sl.elevation), rad2deg.(sl.azimuth),
                  l_tether(sl), force(sl), v_reelout(sl), sl.cycle;
            ylabels=["height [m]", "elevation [°]", "azimuth [°]", "length [m]",
                     "force [N]", "v_ro [m/s]", "cycle [-]"],
            yzoom=0.9, fig="main", ysize=ysize))
    nothing
end

# ---------------------------------------------------------------------------
# _plot_power
# ---------------------------------------------------------------------------
function _plot_power(log; ysize=10, energy=nothing, dt=0.0)
    sl = log.syslog
    if energy === nothing
        v_ro = v_reelout(sl)
        f_  = force(sl)
        energy = similar(v_ro)
        en = 0.0
        for i in eachindex(energy)
            en += f_[i] * v_ro[i] * dt
            energy[i] = en
        end
    end
    display(plotx(sl.time, force(sl), v_reelout(sl), force(sl) .* v_reelout(sl),
                  energy ./ 3600, sl.acc;
            ylabels=["force [N]", L"v_\mathrm{ro}~[m/s]", L"P_\mathrm{m}~[W]",
                     "Energy [Wh]", "acc [m/s^2]"],
            fig="power", ysize=ysize))
    nothing
end

# ---------------------------------------------------------------------------
# _plot_control  (two sub-plots: control + fpc)
# ---------------------------------------------------------------------------
function _plot_control(log; ysize=10)
    sl = log.syslog
    display(plotx(sl.time, rad2deg.(sl.elevation), rad2deg.(sl.azimuth),
                  rad2deg.(wrap2pi.(sl.heading)), force(sl),
                  100*sl.depower, 100*sl.steering, sl.sys_state, sl.cycle, sl.fig_8;
            ylabels=["elevation [°]", "azimuth [°]", "heading [°]", "force [N]",
                     "depower [%]", "steering [%]", "fpp_state", "cycle", "fig8"],
            fig="control", ysize=ysize, yzoom=0.7))
    sleep(0.05)
    display(plotx(sl.time, rad2deg.(sl.elevation), rad2deg.(sl.azimuth),
                  -rad2deg.(wrap2pi.(sl.heading)), 100*sl.depower, 100*sl.steering,
                  rad2deg.(sl.var_07), sl.var_06, sl.sys_state, sl.cycle;
            ylabels=["elevation [°]", "azimuth [°]", "psi [°]", "depower [%]",
                     "steering [%]", "chi_set", "ndi_gain", "fpp_state", "cycle"],
            fig="fpc", ysize=ysize, yzoom=0.7))
    nothing
end

# ---------------------------------------------------------------------------
# _plot_control_II
# ---------------------------------------------------------------------------
function _plot_control_II(log; ysize=10)
    sl = log.syslog
    display(plotx(sl.time, rad2deg.(sl.azimuth), -rad2deg.(wrap2pi.(sl.heading)),
                  100*sl.steering, sl.var_12, rad2deg.(sl.course .- pi),
                  rad2deg.(sl.var_09), rad2deg.(sl.var_10), sl.var_06, sl.sys_state;
            ylabels=["azimuth [°]", "psi [°]", "steering [%]", "c2", "chi",
                     "psi_dot_set", "psi_dot", "ndi_gain", "fpp_state"],
            fig="fpc", ysize=ysize, yzoom=0.7))
    nothing
end

# ---------------------------------------------------------------------------
# _plot_winch_control  (two sub-plots)
# ---------------------------------------------------------------------------
function _plot_winch_control(log; ysize=10)
    sl = log.syslog
    display(plotx(sl.time, rad2deg.(sl.elevation), rad2deg.(sl.azimuth),
                  force(sl), sl.var_04, v_reelout(sl),
                  100*sl.depower, 100*sl.steering, sl.var_03;
            ylabels=["elevation [°]", "azimuth [°]", "force [N]", "set_force",
                     "v_reelout [m/s]", "depower [%]", "steering [%]", "wc_state"],
            fig="winch_control", ysize=ysize))
    display(plot(sl.time, [v_reelout(sl), sl.var_05];
            labels=["v_reelout", "pid2_v_set_out"],
            ylabel="v_reelout [m/s]",
            xlabel="time [s]",
            fig="winch", ysize=ysize))
    nothing
end

# ---------------------------------------------------------------------------
# _plot_aerodynamics
# ---------------------------------------------------------------------------
function _plot_aerodynamics(log; plot_lift_drag=false)
    sl = log.syslog
    if plot_lift_drag
        display(plotx(sl.time, sl.var_08, rad2deg.(sl.AoA), sl.CL2, sl.CD2;
                      ylabels=["LoD [-]", L"AoA~[°]", "CL [-]", "CD [-]"],
                      fig="aerodynamics"))
        display(plotxy(rad2deg.(sl.AoA[2:end]), sl.CL2[2:end];
                      xlabel="AoA [°]", ylabel="CL [-]",
                      fig="CL as function of AoA"))
        display(plotxy(rad2deg.(sl.AoA[2:end]), sl.CD2[2:end];
                      xlabel="AoA [°]", ylabel="CD [-]",
                      fig="CD_tot as function of AoA"))
    else
        display(plotx(sl.time, sl.var_08, rad2deg.(sl.AoA), 100*sl.steering,
                      sl.var_15, rad2deg.(sl.var_16);
                    ylabels=["LoD [-]", L"AoA~[°]", "steering [%]",
                             "yaw_rate [°/s]", L"side\_slip~[°]"],
                    fig="aerodynamics"))
    end
    nothing
end

# ---------------------------------------------------------------------------
# _plot_elev_az variations
# ---------------------------------------------------------------------------
function _plot_elev_az(log)
    sl = log.syslog
    display(plotxy(rad2deg.(sl.azimuth), rad2deg.(sl.elevation);
            ylabel="elevation [°]", xlabel="azimuth [°]", fig="elev_az"))
    nothing
end

function _plot_elev_az_n(log, cycle_number)
    sl = log.syslog
    idx = _cycle_index(sl, cycle_number)
    display(plotxy(rad2deg.(sl.azimuth)[idx:end], rad2deg.(sl.elevation)[idx:end];
            ylabel="elevation [°]", xlabel="azimuth [°]", fig="elev_az"))
    nothing
end

# ---------------------------------------------------------------------------
# _plot_side_view variations
# ---------------------------------------------------------------------------
function _plot_side_view(log)
    display(plotxy(log.x, log.z;
            ylabel="pos_x [m]", xlabel="height [m]", fig="side_view"))
    nothing
end

function _plot_side_view_n(log, cycle_number)
    sl = log.syslog
    idx = _cycle_index(sl, cycle_number)
    display(plotxy(log.x[idx:end], log.z[idx:end];
            ylabel="pos_x [m]", xlabel="height [m]", fig="side_view"))
    nothing
end

# ---------------------------------------------------------------------------
# _plot_front_view_n
# ---------------------------------------------------------------------------
function _plot_front_view_n(log, cycle_number)
    sl = log.syslog
    idx = _cycle_index(sl, cycle_number)
    display(plotxy(log.y[idx:end], log.z[idx:end];
            xlabel="pos_y [m]", ylabel="height [m]", fig="front_view"))
    nothing
end
