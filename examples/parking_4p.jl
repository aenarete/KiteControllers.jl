# parking_4p.jl — Demonstrate the parking controller for a four-point kite model (KPS4).
# Simulates the kite flying to and holding a parked (zero-azimuth) position using the
# new parking controller, with a brief steering disturbance applied mid-flight.
# Displays a real-time 3D viewer and plots the azimuth over time when done.

# activate the test environment if needed
using Pkg
if ! ("MakieControlPlots" ∈ keys(Pkg.project().dependencies))
    Pkg.activate(@__DIR__)
end
using Timers; tic()

using KiteViewers
using MakieControlPlots, KiteViewers, Rotations
using KiteUtils: Settings, load_settings
using KitePodModels: KCU
using KiteModels
using KiteModels: KPS4
using KiteModels: reactivate_host_app

set::Settings = deepcopy(load_settings("system.yaml"))
set.abs_tol=0.00006
set.rel_tol=0.0001
set.v_wind = 10 # v_min1 6-25; v_min2 5.3-30
default_turbulence = get_default_turbulence()
if default_turbulence !== nothing
    set.use_turbulence = default_turbulence
end

include("parking_controller.jl")
import .ParkingControllers as pcm
pcs = pcm.ParkingControllerSettings(dt=0.05)

kcu::KCU = KCU(set)
kps4::KPS4 = KPS4(kcu)
@assert set.sample_freq == 20
dt::Float64 = 1/set.sample_freq
max_turn_rate_cmd = let raw = get(ENV, "MAX_TURN_RATE_CMD", "0.50")
    parsed = tryparse(Float64, raw)
    isnothing(parsed) ? 0.50 : parsed
end

MIN_DEPOWER, DISTURBANCE = if KiteUtils.PROJECT == "system.yaml"
    # result of tuning
    pcs.kp_tr=0.15
    pcs.ki_tr=0.003
    pcs.kp = 1.0
    pcs.ki = 0.025
    pcs.kd = 2.5
    pcs.kd_N = 2
    pcs.c1 = 0.048
    pcs.c2 = 0 # has no big effect, can also be set to zero
    pcs.max_turn_rate_set = 0.20
    pcs.max_turn_rate_cmd = max_turn_rate_cmd
    pcs.max_steering = 0.45
    pcs.max_steering_rate = 1.0
    0.22, 0.1
else
    # result of tuning
    println("not system.yaml")
    pcs.kp_tr=0.035
    pcs.ki_tr=0.0015
    pcs.kp = 1.0
    pcs.ki = 0.025
    pcs.kd = 2.5
    pcs.kd_N = 2
    pcs.c1 = 0.048
    pcs.c2 = 0    # has no big effect, can also be set to zero
    pcs.max_turn_rate_set = 0.20
    pcs.max_turn_rate_cmd = max_turn_rate_cmd
    pcs.max_steering = 0.45
    pcs.max_steering_rate = 1.0
    0.4, 0.4
end
@info "pcs.kp_tr=$(pcs.kp_tr), pcs.ki_tr=$(pcs.ki_tr), pcs.kp=$(pcs.kp), pcs.ki=$(pcs.ki), pcs.max_turn_rate_cmd=$(pcs.max_turn_rate_cmd), MIN_DEPOWER=$(MIN_DEPOWER)"
pc = pcm.ParkingController(pcs)

# the following values can be changed to match your interest
MAX_TIME::Float64 =  60 # was 60
TIME_LAPSE_RATIO  =  6
SHOW_KITE         = true
# end of user parameter section #

viewer::Viewer3D = Viewer3D(SHOW_KITE, "WinchON")

T::Vector{Float64} = zeros(Int64(MAX_TIME/dt))
AZIMUTH::Vector{Float64}       = zeros(Int64(MAX_TIME/dt))
HEADING::Vector{Float64}       = zeros(Int64(MAX_TIME/dt))
SET_STEERING::Vector{Float64}  = zeros(Int64(MAX_TIME/dt))
STEERING::Vector{Float64}      = zeros(Int64(MAX_TIME/dt))
AoA::Vector{Float64}           = zeros(Int64(MAX_TIME/dt))
PSI_DOT::Vector{Float64}       = zeros(Int64(MAX_TIME/dt))
PSI_DOT_SET::Vector{Float64}   = zeros(Int64(MAX_TIME/dt))
NDI_GAIN::Vector{Float64}      = zeros(Int64(MAX_TIME/dt))
V_APP::Vector{Float64}         = zeros(Int64(MAX_TIME/dt))

function simulate(integrator)
    global sys_state
    start_time_ns = time_ns()
    clear_viewer(viewer)
    i=1; j=0; k=0
    GC.gc()
    max_time = 0
    t_gc_tot = 0
    sys_state = SysState(kps4)
    while true
        steering = 0.0
        if i >= 100
            if i == 100
                pc.last_heading = sys_state.heading
            end
            chi_set = pcm.navigate(pc, sys_state.azimuth, sys_state.elevation)
            steering, ndi_gain, psi_dot, psi_dot_set = pcm.calc_steering(pc, sys_state.heading, chi_set; 
                                                                         sys_state.elevation, v_app = sys_state.v_app)
            PSI_DOT[i] = psi_dot
            PSI_DOT_SET[i] = psi_dot_set
            NDI_GAIN[i] = ndi_gain
            V_APP[i] = sys_state.v_app
            time = i * dt
            # disturbance
            if time > 20 && time < 21
                steering = DISTURBANCE
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
        if mod(i, TIME_LAPSE_RATIO) == 0
            if KiteUtils.PROJECT == "system.yaml"
                KiteViewers.update_system(viewer, sys_state; scale = 0.08, kite_scale=3)
            else
                KiteViewers.update_system(viewer, sys_state; scale = 0.08*0.5, kite_scale=3)
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
    integrator = KiteModels.init!(kps4, stiffness_factor=0.5)
    toc()
    simulate(integrator)
    GC.enable(true)
end

function play1()
    if viewer.stop
        Base.invokelatest(play)
        stop(viewer)
    end
end

on(viewer.btn_PLAY.clicks) do _; Base.invokelatest(play1); end

Base.invokelatest(play)
stop(viewer)
p = plotx(T, rad2deg.(AZIMUTH), rad2deg.(HEADING), [100*(SET_STEERING), 100*(STEERING)],
             [rad2deg.(PSI_DOT), rad2deg.(PSI_DOT_SET)], NDI_GAIN, V_APP; 
          xlabel="Time [s]", 
          ylabels=["Azimuth [°]", "Heading [°]", "steering [%]", "psi_dot [°/s]", "NDI_GAIN", "v_app [m/s]"],   
          labels=["azimuth", "heading", ["set_steering", "steering"], ["psi_dot", "psi_dot_set"], "NDI_GAIN", "v_app"],  
          fig="Azimuth, heading, steering and more")
display(p)
reactivate_host_app()
