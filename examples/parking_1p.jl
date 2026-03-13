# parking_1p.jl — Demonstrate the parking controller for a one-point kite model (KPS3).
# Simulates the kite flying to and holding a parked (zero-azimuth) position using the
# new parking controller, with a brief steering disturbance applied mid-flight.
# Displays a real-time 3D viewer and plots the azimuth over time when done.

# activate the test environment if needed
using Pkg
if ! ("ControlPlots" ∈ keys(Pkg.project().dependencies))
    Pkg.activate(@__DIR__)
end
using Timers; tic()

using KiteUtils: Settings, load_settings

set::Settings = deepcopy(load_settings("system.yaml"))
set.abs_tol=0.00006
set.rel_tol=0.0001
set.v_wind = 6.5

using KiteViewers
using KiteModels: reactivate_host_app
using ControlPlots, KiteModels, Rotations
using KiteControllers
using KiteControllers: FPCSettings, FPPSettings, SystemStateControl, WCSettings,
                       on_parking, on_stop, on_winchcontrol

kcu::KCU = KCU(set)
kps::KPS3 = KPS3(kcu)
wcs::WCSettings = WCSettings(true, dt = 1/set.sample_freq)
fcs::FPCSettings = FPCSettings(true, dt = wcs.dt)
fpps::FPPSettings = FPPSettings(true)
u_d0 = 0.01 * set.depower_offset
u_d = 0.01 * set.depowers[1]
ssc::SystemStateControl = SystemStateControl(wcs, fcs, fpps; u_d0, u_d, v_wind = set.v_wind)
dt::Float64 = wcs.dt

# result of tuning
fcs.p = 13.63*0.2
fcs.i = 0.2
fcs.d = 27.75*0.85
MIN_DEPOWER = 0.24

# the following values can be changed to match your interest
MAX_TIME::Float64 = 60
TIME_LAPSE_RATIO  = 6
SHOW_KITE         = true
# end of user parameter section #

viewer::Viewer3D = Viewer3D(SHOW_KITE, "WinchON")

steps = 0
T::Vector{Float64} = zeros(Int64(MAX_TIME/dt))
AZIMUTH::Vector{Float64}       = zeros(Int64(MAX_TIME/dt))

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
            heading = calc_heading(kps; neg_azimuth=true, one_point=false)
            steering = calc_steering(ssc, 0; heading)
            time = i * dt
            # disturbance
            if time > 20 && time < 21
                steering = 0.1
            end
            set_depower_steering(kps.kcu, MIN_DEPOWER, steering)
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
            sys_state.orient = quat2viewer(QuatRotation(sys_state.orient))
            KiteViewers.update_system(viewer, sys_state; scale = 0.08, kite_scale=3, ned=false)
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
        if i==1
            bring_viewer_to_front()
        end
        i += 1
    end
    misses = j/k * 100
    println("\nMissed the deadline for $(round(misses, digits=2)) %. Max time: $(round((max_time*1e-6), digits=1)) ms")
    return div(i, TIME_LAPSE_RATIO)
end

function play()
    global steps
    integrator = KiteModels.init!(kps, stiffness_factor=0.04)
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
    on_winchcontrol(ssc)
end

on(viewer.btn_STOP.clicks) do _; stop(viewer); on_stop(ssc) end
on(viewer.btn_PLAY.clicks) do _; async_play(); end
on(viewer.btn_PARKING.clicks) do _; parking(); end

play()
stop(viewer)

p = plot(T, rad2deg.(AZIMUTH); xlabel="Time [s]", ylabel="Azimuth [deg]")
display(p)
reactivate_host_app()
