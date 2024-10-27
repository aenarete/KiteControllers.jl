# activate the test environment if needed
using Pkg
if ! ("ControlPlots" ∈ keys(Pkg.project().dependencies))
    using TestEnv; TestEnv.activate()
end
using Timers; tic()

using KiteControllers, KiteViewers, KiteModels, ControlPlots, Rotations
set = deepcopy(load_settings("system.yaml"))
set.abs_tol=0.00006
set.rel_tol=0.0001
set.v_wind = 8.5 # v_min1 7.7; v_min2 8.5

kcu::KCU = KCU(set)
kps4::KPS4 = KPS4(kcu)
@assert set.sample_freq == 20
wcs::WCSettings = WCSettings(true; dt = 1/set.sample_freq)
fcs::FPCSettings = FPCSettings(true; dt = wcs.dt)
fpps::FPPSettings = FPPSettings(true)
u_d0 = 0.01 * set.depower_offset
u_d = 0.01 * set.depower
ssc::SystemStateControl = SystemStateControl(wcs, fcs, fpps; u_d0, u_d, v_wind = set.v_wind)
dt::Float64 = wcs.dt

if KiteUtils.PROJECT == "system.yaml"
    # result of tuning
    fcs.p=1.2
    fcs.i=0.04
    fcs.d=13.25*0.95
    MIN_DEPOWER       = 0.22
    fcs.use_chi = false
    @assert fcs.gain == 0.04
else
    # result of tuning
    println("not system.yaml")
    fcs.p=1.05
    fcs.i=0.012
    fcs.d=13.25*2.0
    MIN_DEPOWER       = 0.4
    fcs.use_chi = false
    fcs.gain = 0.04
    fcs.c1 = 0.048
    fcs.c2 = 5.5
end
println("fcs.p=$(fcs.p), fcs.i=$(fcs.i), fcs.d=$(fcs.d), fcs.gain=$(fcs.gain)")

# the following values can be changed to match your interest
MAX_TIME::Float64 = 60 # was 60
TIME_LAPSE_RATIO  =  6
SHOW_KITE         = true
# end of user parameter section #

viewer::Viewer3D = Viewer3D(SHOW_KITE, "WinchON")

steps = 0
T::Vector{Float64} = zeros(Int64(MAX_TIME/dt))
AZIMUTH::Vector{Float64}       = zeros(Int64(MAX_TIME/dt))
HEADING::Vector{Float64}       = zeros(Int64(MAX_TIME/dt))
SET_STEERING::Vector{Float64}  = zeros(Int64(MAX_TIME/dt))
STEERING::Vector{Float64}      = zeros(Int64(MAX_TIME/dt))
AoA::Vector{Float64}           = zeros(Int64(MAX_TIME/dt))

function simulate(integrator)
    global sys_state
    start_time_ns = time_ns()
    clear_viewer(viewer)
    i=1; j=0; k=0
    GC.gc()
    max_time = 0
    t_gc_tot = 0
    sys_state = SysState(kps4)
    on_new_systate(ssc, sys_state)
    while true
        steering = 0.0
        if i > 100
            heading = calc_heading(kps4; neg_azimuth=true, one_point=false)
            steering = calc_steering(ssc, 0; heading)
            time = i * dt
            # disturbance
            if time > 20 && time < 21
                steering = 0.1
            end            
            set_depower_steering(kps4.kcu, MIN_DEPOWER, steering)
        end
        SET_STEERING[i] = steering
        STEERING[i] = get_steering(kps4.kcu)/set.cs_4p
        AoA[i] = kps4.alpha_2
        # execute winch controller
        v_ro = 0.0
        t_sim = @elapsed KiteModels.next_step!(kps4, integrator; set_speed=v_ro, dt=dt)
        if t_sim < 0.3*dt
            t_gc_tot += @elapsed GC.gc(false)
        end
        sys_state = SysState(kps4)
        T[i] = dt * i
        AZIMUTH[i] = sys_state.azimuth
        HEADING[i] = wrap2pi(sys_state.heading)
        on_new_systate(ssc, sys_state)
        if mod(i, TIME_LAPSE_RATIO) == 0
            if KiteUtils.PROJECT == "system.yaml"
                KiteViewers.update_system(viewer, sys_state; scale = 0.08, kite_scale=3)
            else
                KiteViewers.update_system(viewer, sys_state; scale = 0.08*0.5, kite_scale=3)
            end
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
    integrator = KiteModels.init_sim!(kps4, stiffness_factor=0.5)
    toc()
    steps = simulate(integrator)
    GC.enable(true)
end

function play1()
    if viewer.stop
        play()
        stop(viewer)
    end
end

function parking()
    on_parking(ssc)
end

function autopilot()
    on_winchcontrol(ssc)
end

on(viewer.btn_STOP.clicks) do c; stop(viewer); on_stop(ssc) end
on(viewer.btn_PLAY.clicks) do c; play1(); end
on(viewer.btn_PARKING.clicks) do c; parking(); end

play()
stop(viewer)
p = plotx(T, rad2deg.(AZIMUTH), rad2deg.(HEADING), [100*(SET_STEERING), 100*(STEERING)], AoA; 
          xlabel="Time [s]", 
          ylabels=["Azimuth [°]", "Heading [°]", "steering [%]", "AoA [°]"],
          labels=["azimuth", "heading", ["set_steering", "steering", "AoA"]], 
          fig="Azimuth, heading, steering and AoA",)
display(p)
