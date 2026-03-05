# batch_plot.jl
# Command-line plotting tool for kite simulation logs.
# No GUI — select a plot from a text menu.
# Default input file: output/sim_log.arrow
# Usage: julia batch_plot.jl [path/to/file.arrow]

using Pkg
if ! ("ControlPlots" ∈ keys(Pkg.project().dependencies))
    Pkg.activate(@__DIR__)
end

using ControlPlots, KiteControllers, LaTeXStrings, Statistics
using REPL.TerminalMenus

# Hard-coded default log file (override by passing a path as a command-line argument)
const PLOT_FILE = isempty(ARGS) ? joinpath(@__DIR__, "..", "output", "batch-hydra20_600.arrow") : ARGS[1]

# ---------------------------------------------------------------------------
# Helper accessors (same as in plots.jl)
# ---------------------------------------------------------------------------
function l_tether(sl)
    hcat(sl.l_tether...)[1,:]
end

function force(sl)
    hcat(sl.winch_force...)[1,:]
end

function v_reelout(sl)
    hcat(sl.v_reelout...)[1,:]
end

# Derive dt from the log instead of from an app object
function log_dt(sl)
    length(sl.time) > 1 ? sl.time[2] - sl.time[1] : 0.05
end

# ---------------------------------------------------------------------------
# Load helper
# ---------------------------------------------------------------------------
function load_plot_log()
    load_log(basename(PLOT_FILE); path=dirname(PLOT_FILE))
end

# ---------------------------------------------------------------------------
# Plot functions (adapted from plots.jl — no KiteViewers references)
# ---------------------------------------------------------------------------
function plot_main()
    log = load_plot_log()
    sl  = log.syslog
    display(plotx(sl.time, log.z, rad2deg.(sl.elevation), rad2deg.(sl.azimuth),
                  l_tether(sl), force(sl), v_reelout(sl), sl.cycle;
            ylabels=["height [m]", "elevation [°]", "azimuth [°]", "length [m]",
                     "force [N]", "v_ro [m/s]", "cycle [-]"],
            yzoom=0.9, fig="main"))
    nothing
end

function plot_power()
    log = load_plot_log()
    sl  = log.syslog
    display(plotx(sl.time, force(sl), v_reelout(sl), force(sl) .* v_reelout(sl), sl.e_mech, sl.acc;
            ylabels=["force [N]", L"v_\mathrm{ro}~[m/s]", L"P_\mathrm{m}~[W]", "Energy [Wh]", "acc [m/s^2]"],
            fig="power"))
    nothing
end

function plot_control()
    log = load_plot_log()
    sl  = log.syslog
    display(plotx(sl.time, rad2deg.(sl.elevation), rad2deg.(sl.azimuth),
                  rad2deg.(wrap2pi.(sl.heading)), force(sl),
                  100*sl.depower, 100*sl.steering, sl.sys_state, sl.cycle, sl.fig_8;
            ylabels=["elevation [°]", "azimuth [°]", "heading [°]", "force [N]",
                     "depower [%]", "steering [%]", "fpp_state", "cycle", "fig8"],
            fig="control", ysize=10, yzoom=0.7))
    sleep(0.05)
    display(plotx(sl.time, rad2deg.(sl.elevation), rad2deg.(sl.azimuth),
                  -rad2deg.(wrap2pi.(sl.heading)), 100*sl.depower, 100*sl.steering,
                  rad2deg.(sl.var_07), sl.var_06, sl.sys_state, sl.cycle;
            ylabels=["elevation [°]", "azimuth [°]", "psi [°]", "depower [%]",
                     "steering [%]", "chi_set", "ndi_gain", "fpp_state", "cycle"],
            fig="fpc", ysize=10, yzoom=0.7))
    nothing
end

function plot_control_II()
    log = load_plot_log()
    sl  = log.syslog
    display(plotx(sl.time, rad2deg.(sl.azimuth), -rad2deg.(wrap2pi.(sl.heading)),
                  100*sl.steering, sl.var_12, rad2deg.(sl.course .- pi),
                  rad2deg.(sl.var_09), rad2deg.(sl.var_10), sl.var_06, sl.sys_state;
            ylabels=["azimuth [°]", "psi [°]", "steering [%]", "c2", "chi",
                     "psi_dot_set", "psi_dot", "ndi_gain", "fpp_state"],
            fig="fpc", ysize=10, yzoom=0.7))
    nothing
end

function plot_winch_control()
    log = load_plot_log()
    sl  = log.syslog
    display(plotx(sl.time, rad2deg.(sl.elevation), rad2deg.(sl.azimuth),
                  force(sl), sl.var_04, v_reelout(sl),
                  100*sl.depower, 100*sl.steering, sl.var_03;
            ylabels=["elevation [°]", "azimuth [°]", "force [N]", "set_force",
                     "v_reelout [m/s]", "depower [%]", "steering [%]", "wc_state"],
            fig="winch_control", ysize=10))
    display(plot(sl.time, [v_reelout(sl), sl.var_05];
            labels=["v_reelout", "pid2_v_set_out"],
            ylabel="v_reelout [m/s]",
            xlabel="time [s]",
            fig="winch", ysize=10))
    nothing
end

function plot_aerodynamics(plot_lift_drag=false)
    log = load_plot_log()
    sl  = log.syslog
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

function plot_elev_az()
    log = load_plot_log()
    sl  = log.syslog
    display(plotxy(rad2deg.(sl.azimuth), rad2deg.(sl.elevation);
            ylabel="elevation [°]", xlabel="azimuth [°]", fig="elev_az"))
    nothing
end

function plot_elev_az2()
    log = load_plot_log()
    sl  = log.syslog
    index = 1
    for i in 1:length(sl.cycle)
        if sl.cycle[i] == 2; index = i; break; end
    end
    display(plotxy(rad2deg.(sl.azimuth)[index:end], rad2deg.(sl.elevation)[index:end];
            ylabel="elevation [°]", xlabel="azimuth [°]", fig="elev_az"))
    nothing
end

function plot_elev_az3()
    log = load_plot_log()
    sl  = log.syslog
    index = 1
    for i in 1:length(sl.cycle)
        if sl.cycle[i] == 3; index = i; break; end
    end
    display(plotxy(rad2deg.(sl.azimuth)[index:end], rad2deg.(sl.elevation)[index:end];
            ylabel="elevation [°]", xlabel="azimuth [°]", fig="elev_az"))
    nothing
end

function plot_side_view()
    log = load_plot_log()
    display(plotxy(log.x, log.z;
            ylabel="pos_x [m]", xlabel="height [m]", fig="side_view"))
    nothing
end

function plot_side_view2()
    log = load_plot_log()
    sl  = log.syslog
    index = 1
    for i in 1:length(sl.cycle)
        if sl.cycle[i] == 2; index = i; break; end
    end
    display(plotxy(log.x[index:end], log.z[index:end];
            ylabel="pos_x [m]", xlabel="height [m]", fig="side_view"))
    nothing
end

function plot_side_view3()
    log = load_plot_log()
    sl  = log.syslog
    index = 1
    for i in 1:length(sl.cycle)
        if sl.cycle[i] == 3; index = i; break; end
    end
    display(plotxy(log.x[index:end], log.z[index:end];
            ylabel="pos_x [m]", xlabel="height [m]", fig="side_view"))
    nothing
end

function plot_front_view3()
    log = load_plot_log()
    sl  = log.syslog
    index = 1
    for i in 1:length(sl.cycle)
        if sl.cycle[i] == 3; index = i; break; end
    end
    display(plotxy(log.y[index:end], log.z[index:end];
            xlabel="pos_y [m]", ylabel="height [m]", fig="front_view"))
    nothing
end

function plot_timing()
    log = load_plot_log()
    sl  = log.syslog
    dt  = log_dt(sl)
    display(plotx(sl.time, sl.t_sim, 100*sl.steering, 100*sl.depower;
                  ylabels=["t_sim [ms]", "steering [%]", "depower [%]"],
                  fig="timing"))
    println("Mean    time per timestep: $(mean(sl.t_sim)) ms")
    println("Maximum time per timestep: $(maximum(sl.t_sim[10:end])) ms")
    index = Int64(round(12 / dt))
    println("Maximum for t>12s        : $(maximum(sl.t_sim[index:end])) ms")
    nothing
end

function plot_timing2()
    log = load_plot_log()
    sl  = log.syslog
    display(plot(sl.time, sl.t_sim; ylabel="t_sim [ms]", fig="timing2"))
    nothing
end

# ---------------------------------------------------------------------------
# Interactive menu (REPL.TerminalMenus RadioMenu)
# ---------------------------------------------------------------------------
const MENU_ITEMS = [
    ("plot_main",          plot_main),
    ("plot_power",         plot_power),
    ("plot_control",       plot_control),
    ("plot_control_II",    plot_control_II),
    ("plot_winch_control", plot_winch_control),
    ("plot_aerodynamics",  () -> plot_aerodynamics(false)),
    ("plot_elev_az",       plot_elev_az),
    ("plot_elev_az2",      plot_elev_az2),
    ("plot_elev_az3",      plot_elev_az3),
    ("plot_side_view",     plot_side_view),
    ("plot_side_view2",    plot_side_view2),
    ("plot_side_view3",    plot_side_view3),
    ("plot_front_view3",   plot_front_view3)
]

const OPTIONS = [item[1] for item in MENU_ITEMS]
push!(OPTIONS, "quit")

function run_menu()
    println("\nLog file: $PLOT_FILE")
    active = true
    while active
        menu   = RadioMenu(OPTIONS, pagesize=8)
        choice = request("\nChoose plot to display or `q` to quit: ", menu)
        if choice != -1 && choice != length(OPTIONS)
            name, fn = MENU_ITEMS[choice]
            println("Running $name …")
            try
                fn()
            catch e
                println("Error in $name: $e")
            end
        else
            println("Left menu. Press <ctrl><d> to quit Julia!")
            active = false
        end
    end
end

run_menu()
