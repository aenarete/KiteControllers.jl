# activate the test environment if needed
using Pkg
if ! ("Plots" ∈ keys(Pkg.project().dependencies))
    using TestEnv; TestEnv.activate()
end
using Timers; tic()

using KiteControllers, KiteViewers, KiteModels, StatsBase

if ! @isdefined kcu;    const kcu = KCU(se());   end
if ! @isdefined kps4;   const kps4 = KPS4(kcu); end

wcs = WCSettings(); update(wcs); wcs.dt = 1/se().sample_freq
fcs::FPCSettings = FPCSettings(); fcs.dt = wcs.dt
fpps::FPPSettings = FPPSettings()
ssc::SystemStateControl = SystemStateControl(wcs, fcs, fpps)
dt::Float64 = wcs.dt

# the following values can be changed to match your interest
MAX_TIME::Float64 = 460
TIME_LAPSE_RATIO  = 4
SHOW_KITE         = true
# end of user parameter section #

phi_set = 21.48
# on_control_command(ssc.fpp.fpca.fpc, attractor=[deg2rad(phi_set), deg2rad(51.88)])
# on_control_command(ssc.fpp.fpca.fpc, psi_dot_set=-23.763, radius=-4.35)

viewer::Viewer3D = Viewer3D(SHOW_KITE)

steps = 0
if ! @isdefined T;        const T = zeros(Int64(MAX_TIME/dt)); end
if ! @isdefined DELTA_T;  const DELTA_T = zeros(Int64(MAX_TIME/dt)); end
if ! @isdefined STEERING; const STEERING = zeros(Int64(MAX_TIME/dt)); end
if ! @isdefined DEPOWER_; const DEPOWER_ = zeros(Int64(MAX_TIME/dt)); end
LAST_I::Int64=0

function simulate(integrator)
    global LAST_I
    start_time_ns = time_ns()
    clear_viewer(viewer)
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
            if i == 200
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
        if i*dt > MAX_TIME break end
    end
    misses = j/k * 100
    println("\nMissed the deadline for $(round(misses, digits=2)) %. Max time: $(round((max_time*1e-6), digits=1)) ms")
    return div(i, TIME_LAPSE_RATIO)
end

function play()
    global steps
    viewer.stop=false
    integrator = KiteModels.init_sim!(kps4, stiffness_factor=0.04)
    toc()
    steps = simulate(integrator)
    GC.enable(true)
end

function async_play()
    if viewer.stop
        @async begin
            play()
            stop(viewer)
        end
    end
end

function parking()
    viewer.stop=false
    on_parking(ssc)
end

function autopilot()
    on_autopilot(ssc)
end

on_stop(ssc)
parking()
clear!(kps4)
on(viewer.btn_PLAY.clicks) do c; viewer.stop=false; end
# on(viewer.btn_STOP.clicks) do c; on_stop(ssc) end
on(viewer.btn_PARKING.clicks) do c; parking(); end
on(viewer.btn_AUTO.clicks) do c; autopilot(); end

play()
stop(viewer)
KiteViewers.GLMakie.closeall()

GC.enable(true)

if maximum(DELTA_T) > 0 && haskey(ENV, "PLOT")
    include("../test/plot.jl")
    plotx(T[1:LAST_I], DELTA_T[1:LAST_I], 100*STEERING[1:LAST_I], 100*DEPOWER_[1:LAST_I], 
        labels=["t_sim [ms]", "steering [%]", "depower [%]"], 
        fig="simulation_timing")
    println("Mean    time per timestep: $(mean(DELTA_T)) ms")
    println("Maximum time per timestep: $(maximum(DELTA_T)) ms")
    index=Int64(round(12/dt))
    println("Maximum for t>12s        : $(maximum(DELTA_T[index:end])) ms")
    plt.pause(0.01)
    plt.show(block=true)
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
