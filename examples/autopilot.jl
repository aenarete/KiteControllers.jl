# activate the test environment if needed
using Pkg
if ! ("Plots" ∈ keys(Pkg.project().dependencies))
    using TestEnv; TestEnv.activate()
end
using Timers; tic()

using KiteControllers, KiteViewers, KiteModels

const ssc = SystemStateControl()

# change this to KPS3 or KPS4
const Model = KPS4

if ! @isdefined kcu;    const kcu = KCU(se());   end
if ! @isdefined kps4;   const kps4 = Model(kcu); end

# the following values can be changed to match your interest
dt = 0.05
if ! @isdefined MAX_TIME; MAX_TIME=3600; end
TIME_LAPSE_RATIO = 1
SHOW_KITE = true
# end of user parameter section #

if ! @isdefined viewer; const viewer = Viewer3D(SHOW_KITE); end

steps=0

function update_system2(kps)
    sys_state = SysState(kps)
    KiteViewers.update_system(viewer, sys_state; scale = 0.08, kite_scale=3)
end 

function simulate(integrator)
    start_time_ns = time_ns()
    clear_viewer(viewer)
    i=1
    j=0; k=0
    GC.gc()
    max_time = 0
    t_gc_tot = 0
    while true
        v_ro = 0.0
        t_sim = @elapsed KiteModels.next_step!(kps4, integrator, v_ro=v_ro, dt=dt)
        if t_sim < 0.3*dt
            t_gc_tot += @elapsed GC.gc(false)
        end
        if mod(i, TIME_LAPSE_RATIO) == 0 
            update_system2(kps4) 
            end_time_ns = time_ns()
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
            time_tot = end_time_ns - start_time_ns
            start_time_ns = time_ns()
            t_gc_tot = 0
        end
        if viewer.stop break end
        if i*dt > MAX_TIME break end
        i += 1
    end
    misses = j/k * 100
    println("\nMissed the deadline for $(round(misses, digits=2)) %. Max time: $(round((max_time*1e-6), digits=1)) ms")
    return div(i, TIME_LAPSE_RATIO)
end

function play()
    global steps
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
    on_parking(ssc)
end

function autopilot()
    on_autopilot(ssc)
end

on(viewer.btn_PLAY.clicks) do c; async_play(); end
on(viewer.btn_STOP.clicks) do c; stop(viewer); on_stop(ssc) end
on(viewer.btn_PARKING.clicks) do c; parking(); end
on(viewer.btn_AUTO.clicks) do c; autopilot(); end

play()
stop(viewer)
