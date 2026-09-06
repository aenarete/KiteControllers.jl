using Pkg
if ! ("MakieControlPlots" ∈ keys(Pkg.project().dependencies))
    Pkg.activate(@__DIR__)
end

using KiteControllers
using KiteUtils: load_log
using MakieControlPlots

const OUTPUT_DIR::String = "output"

# Plot force vs time from the last recorded log file
let
    log_path = joinpath(OUTPUT_DIR, "last_sim_log")
    if isfile(log_path * ".arrow")
        log = load_log(basename(log_path); path=dirname(log_path))
        sl = log.syslog
        force = getindex.(sl.winch_force, 1)
        v_ro = getindex.(sl.v_reelout, 1)
        power = force .* v_ro
        p1 = MakieControlPlots.plot(sl.time, force;
                  xlabel="time [s]",
                  ylabel="force [N]",
                  fig="loading")
        display(p1)
        p2 = MakieControlPlots.plot(sl.time, power;
                  xlabel="time [s]",
                  ylabel="power [W]",
                  fig="power")
        display(p2)
        println("Plotted force and power vs time from: $log_path")
    else
        println("No log file found at: $(log_path).arrow")
    end
end