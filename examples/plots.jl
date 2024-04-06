function plot_timing()
    log = load_log(basename(KiteViewers.plot_file[]))
    sl  = log.syslog
    display(ControlPlots.plotx(sl.time, sl.t_sim, 100*sl.steering, 100*sl.depower;
                               ylabels=["t_sim [ms]", "steering [%]","depower [%]"],
                               fig="timing"))
    println("Mean    time per timestep: $(mean(sl.t_sim)) ms")
    println("Maximum time per timestep: $(maximum(sl.t_sim[10:end])) ms")
    index = Int64(round(12/app.dt))
    println("Maximum for t>12s        : $(maximum(sl.t_sim[index:end])) ms")
    nothing
end

function plot_main()
    log = load_log(basename(KiteViewers.plot_file[]))
    sl  = log.syslog
    display(plotx(log.syslog.time, log.z, rad2deg.(sl.elevation), rad2deg.(sl.azimuth), sl.l_tether, sl.force, 
                  sl.v_reelout, sl.var_01;
        ylabels=["height [m]", "elevation [°]", "azimuth [°]", "length [m]", "force [N]", "v_ro [m/s]", "cycle [-]"],
        fig="main"))
     nothing
end

function plot_power()
    log = load_log(basename(KiteViewers.plot_file[]))
    sl  = log.syslog
    energy = similar(sl.v_reelout)
    en=0.0
    for i in eachindex(energy)
        en +=  sl.force[i]*sl.v_reelout[i]*app.dt
        energy[i] = en
    end
    display(plotx(log.syslog.time, sl.force, sl.v_reelout, sl.force.*sl.v_reelout, energy./3600;
            ylabels=["force [N]", L"v_\mathrm{ro}~[m/s]", L"P_\mathrm{m}~[W]", "Energy [Wh]"],
            fig="power"))
    nothing
end

function plot_control()
    log = load_log(basename(KiteViewers.plot_file[]))
    sl  = log.syslog
    display(plotx(log.syslog.time, rad2deg.(sl.elevation), rad2deg.(sl.azimuth), 100*sl.depower, 100*sl.steering, sl.sys_state, sl.var_02;
            ylabels=["elevation [°]", "azimuth [°]", "depower [%]", "steering [%]", "fpp_state", "fig8"],
            fig="control"))
    nothing
end

function plot_elev_az()
    log = load_log(basename(KiteViewers.plot_file[]))
    sl  = log.syslog
    display(plotxy(rad2deg.(sl.azimuth), rad2deg.(sl.elevation);
            ylabel="elevation [°]",
            xlabel="azimuth [°]",
            fig="elev_az"))
    nothing
end

function plot_elev_az2()
    log = load_log(basename(KiteViewers.plot_file[]))
    sl  = log.syslog
    index=1
    for i in 1:length(sl.var_01)
        if sl.var_01[i] == 2
            index=i
            break
        end
    end
    display(plotxy(rad2deg.(sl.azimuth)[index:end], rad2deg.(sl.elevation)[index:end];
            ylabel="elevation [°]",
            xlabel="azimuth [°]",
            fig="elev_az"))
    nothing
end

function plot_side_view()
    log = load_log(basename(KiteViewers.plot_file[]))
    display(plotxy(log.x, log.z;
    ylabel="pos_x [m]",
    xlabel="height [m]",
    fig="side_view"))
    nothing
end

function plot_side_view2()
    log = load_log(basename(KiteViewers.plot_file[]))
    index=1
    for i in 1:length(sl.var_01)
        if sl.var_01[i] == 2
            index=i
            break
        end
    end
    display(plotxy(log.x[index:end], log.z[index:end];
    ylabel="pos_x [m]",
    xlabel="height [m]",
    fig="side_view"))
    nothing
end