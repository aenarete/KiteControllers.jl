# activate the test environment if needed
using Pkg
if ! ("Plots" ∈ keys(Pkg.project().dependencies))
    using TestEnv; TestEnv.activate()
end
using Timers; tic()

using KiteControllers, KiteViewers, KiteModels, StatsBase, ControlPlots, NativeFileDialog

kcu::KCU   = KCU(se())
kps4::KPS4 = KPS4(kcu)

wcs = WCSettings(); update(wcs); wcs.dt = 1/se().sample_freq
fcs::FPCSettings = FPCSettings(); fcs.dt = wcs.dt
fpps::FPPSettings = FPPSettings()
ssc::SystemStateControl = SystemStateControl(wcs, fcs, fpps)
dt::Float64 = wcs.dt

function init_globals()
    global kcu, kps4, wcs, fcs, fpps, ssc
    kcu   = KCU(se())
    kps4 = KPS4(kcu)
    wcs = WCSettings(); update(wcs); wcs.dt = 1/se().sample_freq
    fcs = FPCSettings(); fcs.dt = wcs.dt
    fpps = FPPSettings()
    ssc = SystemStateControl(wcs, fcs, fpps)
    KiteViewers.plot_file[]="data/last_plot.jld2"
end

# the following values can be changed to match your interest
MAX_TIME::Float64 = 460
TIME_LAPSE_RATIO  = 4
SHOW_KITE         = true
# end of user parameter section #

phi_set = 21.48
# on_control_command(ssc.fpp.fpca.fpc, attractor=[deg2rad(phi_set), deg2rad(51.88)])
# on_control_command(ssc.fpp.fpca.fpc, psi_dot_set=-23.763, radius=-4.35)

viewer::Viewer3D = Viewer3D(SHOW_KITE)
PARKING::Bool = false

steps = 0
STEPS::Int64 = Int64(MAX_TIME/dt)
if ! @isdefined T;        const T = zeros(STEPS); end
if ! @isdefined DELTA_T;  const DELTA_T = zeros(STEPS); end
if ! @isdefined STEERING; const STEERING = zeros(STEPS); end
if ! @isdefined DEPOWER_; const DEPOWER_ = zeros(STEPS); end
LAST_I::Int64=0
logger::Logger = Logger(se().segments + 5, STEPS) 

function simulate(integrator, stopped=true)
    global LAST_I
    start_time_ns = time_ns()
    clear_viewer(viewer)
    KiteViewers.running[] = ! stopped
    viewer.stop = stopped
    if ! stopped
        set_status(viewer, "ssParking")
    end
    i=1
    j=0; k=0
    GC.gc()
    GC.gc()
    if Sys.total_memory()/1e9 > 24 && MAX_TIME < 500
        GC.enable(false)
    end
    max_time = 0
    t_gc_tot = 0
    sys_state = SysState(kps4)
    on_new_systate(ssc, sys_state)
    KiteViewers.update_system(viewer, sys_state; scale = 0.04/1.1, kite_scale=6.6)
    log!(logger, sys_state)
    while true
        if viewer.stop
            sleep(dt)
        else
            if i > 100
                dp = KiteControllers.get_depower(ssc)
                if dp < 0.22 dp = 0.22 end
                steering = calc_steering(ssc)
                set_depower_steering(kps4.kcu, dp, steering)
            end
            if i == 200 && ! PARKING
                on_autopilot(ssc)
            end
            # execute winch controller
            v_ro = calc_v_set(ssc)
            #
            t_sim = @elapsed KiteModels.next_step!(kps4, integrator, v_ro=v_ro, dt=dt)
            sys_state = SysState(kps4)
            if i <= length(T)
                T[i] = dt * i
                if i > 10/dt
                    DELTA_T[i]  = t_sim * 1000
                    STEERING[i] = sys_state.steering
                    DEPOWER_[i] = sys_state.depower
                    LAST_I=i
                end
            end
            on_new_systate(ssc, sys_state)
            log!(logger, sys_state)
            if mod(i, TIME_LAPSE_RATIO) == 0 
                KiteViewers.update_system(viewer, sys_state; scale = 0.04/1.1, kite_scale=6.6)
                set_status(viewer, String(Symbol(ssc.state)))
                # turn garbage collection back on if we are short of memory
                if Sys.free_memory()/1e9 < 2.0
                    GC.enable(true)
                end
                wait_until(start_time_ns + 1e9*dt, always_sleep=true) 
                mtime = 0
                if i > 10/dt 
                    # if we missed the deadline by more than 5 ms
                    mtime = time_ns() - start_time_ns
                    if mtime > dt*1e9 + 5e6
                        print(".")
                        j += 1
                    end
                    k +=1
                end
                if mtime > max_time
                    max_time = mtime
                end            
                start_time_ns = time_ns()
                t_gc_tot = 0
            end
            i += 1
        end
        if ! isopen(viewer.fig.scene) break end
        if KiteViewers.status[] == "Stopped" && i > 10 break end
        if i*dt > MAX_TIME break end
    end
    misses = j/k * 100
    println("\nMissed the deadline for $(round(misses, digits=2)) %. Max time: $(round((max_time*1e-6), digits=1)) ms")
    return div(i, TIME_LAPSE_RATIO)
end

function play(stopped=false)
    global steps, kcu, kps4, wcs, fcs, fpps, ssc
    while isopen(viewer.fig.scene)
        init_globals()
        on_parking(ssc)
        integrator = KiteModels.init_sim!(kps4, stiffness_factor=0.04)
        toc()
        steps = simulate(integrator, stopped)
        stopped = ! viewer.sw.active[]
        GC.enable(true)
        if @isdefined __PRECOMPILE__
            break
        end
        plot_timing()
    end
end

function parking()
    global PARKING
    PARKING = true
    viewer.stop=false
    on_parking(ssc)
end

function autopilot()
    global PARKING
    PARKING = false
    viewer.stop=false
    on_autopilot(ssc)
end

function stop_()
    println("Stopping...")
    on_stop(ssc)
    clear!(kps4)
    clear_viewer(viewer)
end

stop_()
on(viewer.btn_PARKING.clicks) do c; parking(); end
on(viewer.btn_AUTO.clicks) do c; autopilot(); end
on(viewer.btn_STOP.clicks) do c; stop_(); end
on(viewer.btn_PLAY.clicks) do c;
    global PARKING
    if ! viewer.stop
        PARKING = false
    end
end

function show_plot()
    filename = KiteViewers.plot_file[]
    res      = load(replace(filename, "~" => homedir()))
    if length(res.X) > 0
        display(res)
        plt.pause(0.01)
        plt.show(block=false)
    end
    nothing
end

function load_plot()
    @async begin 
        filename = fetch(Threads.@spawn pick_file("data"; filterlist="jld2"))
        if filename != ""
            short_filename = replace(filename, homedir() => "~")
            KiteViewers.plot_file[] = short_filename
        end
    end
end

function save_plot()
    @async begin 
        filename = fetch(Threads.@spawn save_file("data"; filterlist="jld2"))
        if filename != ""
            res = load("data/last_plot.jld2")
            ControlPlots.save(filename, res)
            KiteViewers.set_status(viewer, "Saved plot as:")
            KiteViewers.plot_file[] = replace(filename, homedir() => "~")
        end
    end
end

include("logging.jl")

function plot_main()
    log = short_log(logger)
    sl = log.syslog
    println(length(log.syslog.time))
    display(ControlPlots.plotx(log.syslog.time, log.z, rad2deg.(sl.elevation), sl.azimuth, sl.l_tether, sl.force, sl.v_reelout;
            ylabels=["height [m]", "elevation [°]", "azimuth [°]", "length [m]", "force [N]", "v_ro [m/s]"]))
    plt.pause(0.01)
    plt.show(block=false)
end

function plot_timing()
    if maximum(DELTA_T) > 0
        res=plotx(T[1:LAST_I], DELTA_T[1:LAST_I], 100*STEERING[1:LAST_I], 100*DEPOWER_[1:LAST_I], 
            ylabels=["t_sim [ms]", "steering [%]", "depower [%]"], 
            fig="simulation_timing")
        println("Mean    time per timestep: $(mean(DELTA_T)) ms")
        println("Maximum time per timestep: $(maximum(DELTA_T)) ms")
        index = Int64(round(12/dt))
        println("Maximum for t>12s        : $(maximum(DELTA_T[index:end])) ms")
        KiteViewers.plot_file[]="data/last_plot.jld2"
        save(KiteViewers.plot_file[], res)
    end
    nothing
end

on(viewer.btn_OK.clicks) do c
    println(viewer.menu.i_selected[])
    println(viewer.menu.selection[])
    if viewer.menu.i_selected[] == 1
        show_plot()
    elseif viewer.menu.i_selected[] == 2
        load_plot()
    elseif viewer.menu.i_selected[] == 3    
        save_plot()
    end
end

on(viewer.menu.i_selected) do c
    if c == 3
        save_plot()
    elseif c ==2
        load_plot()
    elseif c == 1
        show_plot()
    end
end

if @isdefined __PRECOMPILE__
    MAX_TIME = 30
    play(false)
else
    play(true)
end
stop_()
KiteViewers.GLMakie.closeall()

GC.enable(true)

if maximum(DELTA_T) > 0 && haskey(ENV, "PLOT") && ! @isdefined __PRECOMPILE__
    plot_timing()
end



# GC disabled, Ryzen 7950X, 4x realtime, GMRES
# abs_tol: 0.0006, rel_tol: 0.001
# Missed the deadline for 0.04 %. Max time: 160.4 ms
#     Mean    time per timestep: 3.1066040097826084 ms
#     Maximum time per timestep: 11.13074 ms
#     Maximum for t>12s        : 11.13074 ms

# GC disabled, Ryzen 7950X, 4x realtime, GMRES
# abs_tol: 0.0003, rel_tol: 0.0005
# Missed the deadline for 0.04 %. Max time: 172.1 ms
#     Mean    time per timestep: 3.5648891855434783 ms
#     Maximum time per timestep: 14.024168999999999 ms
#     Maximum for t>12s        : 14.024168999999999 ms
