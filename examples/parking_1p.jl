# activate the test environment if needed
using Pkg
if ! ("ControlPlots" ∈ keys(Pkg.project().dependencies))
    using TestEnv; TestEnv.activate()
end
using Timers; tic()

using KiteUtils
set = deepcopy(load_settings("system.yaml"))
set.abs_tol=0.00000006
set.rel_tol=0.0000001

using KiteControllers, KiteViewers, KiteModels, ControlPlots

kcu::KCU = KCU(set)
kps::KPS3 = KPS3(kcu)
wcs::WCSettings = WCSettings(dt = 1/set.sample_freq)
fcs::FPCSettings = FPCSettings(dt = wcs.dt)
fpps::FPPSettings = FPPSettings()
u_d0 = 0.01 * set.depower_offset
u_d = 0.01 * set.depower
ssc::SystemStateControl = SystemStateControl(wcs, fcs, fpps; u_d0, u_d)
dt::Float64 = wcs.dt

# result of tuning, factor 0.9 to increase robustness
fcs.p = 13.63*0.9
fcs.i = 0.0
fcs.d = 27.75*0.9

# the following values can be changed to match your interest
MAX_TIME::Float64 = 60
TIME_LAPSE_RATIO  = 1
SHOW_KITE         = true
# end of user parameter section #

viewer::Viewer3D = Viewer3D(SHOW_KITE, "WinchON")

steps = 0
if ! @isdefined T;       const T = zeros(Int64(MAX_TIME/dt)); end
if ! @isdefined AZIMUTH; const AZIMUTH = zeros(Int64(MAX_TIME/dt)); end

function simulate(integrator)
    start_time_ns = time_ns()
    clear_viewer(viewer)
    i=1; j=0; k=0
    GC.gc()
    max_time = 0
    t_gc_tot = 0
    sys_state = SysState(kps)
    on_new_systate(ssc, sys_state)
    while true
        if i > 100
            depower = KiteControllers.get_depower(ssc)
            if depower < 0.22; depower = 0.22; end
            steering = calc_steering(ssc, 0)
            time = i * dt
            # disturbance
            if time > 20 && time < 21
                steering = 0.1
            end
            set_depower_steering(kps.kcu, depower, steering)
        end  
        v_ro = 0.0
        t_sim = @elapsed KiteModels.next_step!(kps, integrator; set_speed=v_ro, dt=dt)
        if t_sim < 0.3*dt
            t_gc_tot += @elapsed GC.gc(false)
        end
        sys_state = SysState(kps)
        T[i] = dt * i
        AZIMUTH[i] = sys_state.azimuth        
        on_new_systate(ssc, sys_state)
        if mod(i, TIME_LAPSE_RATIO) == 0
            KiteViewers.update_system(viewer, sys_state; scale = 0.08, kite_scale=3)
            set_status(viewer, String(Symbol(ssc.state)))
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
        if ! isopen(viewer.fig.scene) break end
        if i*dt >= MAX_TIME break end
        i += 1
    end
    misses = j/k * 100
    println("\nMissed the deadline for $(round(misses, digits=2)) %. Max time: $(round((max_time*1e-6), digits=1)) ms")
    return div(i, TIME_LAPSE_RATIO)
end

function play()
    global steps
    integrator = KiteModels.init_sim!(kps, stiffness_factor=0.04)
    toc()
    try
        steps = simulate(integrator)
    catch e
        if isa(e, AssertionError)
            println("AssertionError! Halting simulation.")
        else
            println("Exception! Halting simulation.")
        end
    end
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
    on_winchcontrol(ssc)
end

on(viewer.btn_STOP.clicks) do c; stop(viewer); on_stop(ssc) end
on(viewer.btn_PLAY.clicks) do c; async_play(); end
on(viewer.btn_PARKING.clicks) do c; parking(); end

play()
stop(viewer)

p = plot(T, rad2deg.(AZIMUTH); xlabel="Time [s]", ylabel="Azimuth [deg]")
display(p)
