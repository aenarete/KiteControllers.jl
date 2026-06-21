# Shared plot functions (core logic) live in plot_functions.jl
include("plot_functions.jl")
using KiteViewers

# ---------------------------------------------------------------------------
# Helpers for log loading (specific to the interactive GUI)
# ---------------------------------------------------------------------------
function fulldir(name)
    if occursin("~", name)
        return replace(dirname(name), "~" => homedir())
    else
        return joinpath(pwd(), dirname(name))
    end
end

function resolved_log_file(name)
    log_path = joinpath(fulldir(name), basename(name))
    if isfile(log_path)
        return log_path
    elseif !endswith(log_path, ".arrow") && isfile(log_path * ".arrow")
        return log_path * ".arrow"
    else
        return nothing
    end
end

function log_file_exists()
    log_path = resolved_log_file(KiteViewers.plot_file[])
    log_path !== nothing && return true
    println("Log file not found: $(joinpath(fulldir(KiteViewers.plot_file[]), basename(KiteViewers.plot_file[])))")
    return false
end

function _load_log()
    load_log(basename(KiteViewers.plot_file[]); path=fulldir(KiteViewers.plot_file[]))
end

# ---------------------------------------------------------------------------
# Wrappers — load the log from the interactive viewer then delegate
# ---------------------------------------------------------------------------
function plot_main()
    log_file_exists() || return
    _plot_main(_load_log(); ysize=16)
end

function plot_power()
    log_file_exists() || return
    _plot_power(_load_log(); ysize=16, dt=app.dt)
end

function plot_control()
    log_file_exists() || return
    _plot_control(_load_log(); ysize=16)
end

function plot_control_II()
    log_file_exists() || return
    _plot_control_II(_load_log(); ysize=16)
end

function plot_winch_control()
    log_file_exists() || return
    _plot_winch_control(_load_log(); ysize=14)
end

function plot_aerodynamics(plot_lift_drag=false)
    log_file_exists() || return
    _plot_aerodynamics(_load_log(); plot_lift_drag=plot_lift_drag)
end

function plot_elev_az()
    log_file_exists() || return
    _plot_elev_az(_load_log())
end

function plot_elev_az2()
    log_file_exists() || return
    _plot_elev_az_n(_load_log(), 2)
end

function plot_elev_az3()
    log_file_exists() || return
    _plot_elev_az_n(_load_log(), 3)
end

function plot_side_view()
    log_file_exists() || return
    _plot_side_view(_load_log())
end

function plot_side_view2()
    log_file_exists() || return
    _plot_side_view_n(_load_log(), 2)
end

function plot_side_view3()
    log_file_exists() || return
    _plot_side_view_n(_load_log(), 3)
end

function plot_front_view3()
    log_file_exists() || return
    _plot_front_view_n(_load_log(), 3)
end

# ---------------------------------------------------------------------------
# Timing plots (only in the interactive GUI)
# ---------------------------------------------------------------------------
function plot_timing()
    log_file_exists() || return
    log = _load_log()
    sl  = log.syslog
    time_limit = app.dt/app.set.time_lapse
    t_sim = collect(sl.t_sim)
    tl = fill(time_limit * 1000, length(t_sim))
    display(MakieControlPlots.plotx(sl.time, [t_sim, tl], 100*sl.steering, 100*sl.depower;
                               ylabels=["t_sim [ms]", "steering [%]","depower [%]"],
                               labels=[["t_sim", "time_limit"], "", ""],
                               fig="timing"))
    println("Time limit:                $(time_limit*1000) ms")
    println("Mean    time per timestep: $(mean(sl.t_sim)) ms")
    println("Maximum time per timestep: $(maximum(sl.t_sim[10:end])) ms")
    index = Int64(round(12/app.dt))
    println("Maximum for t>12s        : $(maximum(sl.t_sim[index:end])) ms")
    nothing
end

function plot_timing2()
    log_file_exists() || return
    log = _load_log()
    sl  = log.syslog
    time_limit = app.dt/app.set.time_lapse
    t_sim = collect(sl.t_sim)
    tl = fill(time_limit * 1000, length(t_sim))
    display(MakieControlPlots.plot(sl.time, [t_sim, tl], ylabel="t_sim [ms]", labels=["t_sim","time_limit"], fig="timing2"))
    nothing
end