# park the kind while the wind direction changes
using Pkg
if ! ("MakieControlPlots" ∈ keys(Pkg.project().dependencies))
    Pkg.activate(@__DIR__)
end
using Timers; tic()
using LinearAlgebra

using MakieControlPlots, KiteControllers, KiteModels, KiteViewers, Rotations, Statistics
using KiteUtils: Settings, load_settings
using KiteModels: reactivate_host_app

CREATE_VIDEO = false
PLOT_RATES = false

set::Settings = if haskey(ENV, "USE_V9")
    deepcopy(load_settings("system_v9.yaml"))
else
    deepcopy(load_settings("system.yaml"))
end
default_turbulence = get_default_turbulence()
if default_turbulence !== nothing
    set.use_turbulence = default_turbulence
end
set.abs_tol=0.00006
set.rel_tol=0.0001
set.sample_freq = 20

include("parking_controller.jl")
import .ParkingControllers as pcm
pcs = pcm.ParkingControllerSettings(dt=0.05)

kcu::KCU = KCU(set)
kps4::KPS4 = KPS4(kcu)
@assert set.sample_freq == 20
wcs::WCSettings = WCSettings(dt = 1/set.sample_freq)
@assert wc_settings() == "wc_settings.yaml"
update(wcs); wcs.dt = 1/set.sample_freq
fcs::FPCSettings = FPCSettings(dt = wcs.dt)
@assert fpc_settings() == "fpc_settings.yaml"
update(fcs); fcs.dt = wcs.dt
fpps::FPPSettings = FPPSettings()
@assert fpp_settings() == "fpp_settings.yaml"
update(fpps)
u_d0 = 0.01 * set.depower_offset
u_d = 0.01 * set.depowers[1]
ssc::SystemStateControl = SystemStateControl(wcs, fcs, fpps; u_d0, u_d, v_wind = set.v_wind)
dt::Float64 = wcs.dt

max_turn_rate_cmd = let raw = get(ENV, "MAX_TURN_RATE_CMD", "0.50")
    parsed = tryparse(Float64, raw)
    isnothing(parsed) ? 0.50 : parsed
end
@info "MAX_TURN_RATE_CMD=$(max_turn_rate_cmd)"


MIN_DEPOWER = if KiteUtils.PROJECT == "system.yaml"
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
    0.22
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
    0.4
end

pc = pcm.ParkingController(pcs)

# the following values can be changed to match your interest
MAX_TIME::Float64 = 120
TIME_LAPSE_RATIO  =  6
SHOW_KITE         = true
# For position and velocity vectors of the model the ENU (East North Up) 
UPWIND_DIR        = -pi/2 # the direction the wind is coming from.
UPWIND_DIR2       = -pi/2+deg2rad(90)     # Zero is at north; clockwise positive
# end of user parameter section #

viewer::Viewer3D = Viewer3D(SHOW_KITE, "WinchON")

steps = 0
T::Vector{Float64}             = zeros(Int64(MAX_TIME/dt))
AZIMUTH::Vector{Float64}       = zeros(Int64(MAX_TIME/dt))
AZIMUTH_EAST::Vector{Float64}  = zeros(Int64(MAX_TIME/dt))
UPWIND_DIR_::Vector{Float64}   = zeros(Int64(MAX_TIME/dt))
AV_UPWIND_DIR::Vector{Float64} = zeros(Int64(MAX_TIME/dt))
V_WIND_KITE::Vector{Float64}   = zeros(Int64(MAX_TIME/dt))
FORCE::Vector{Float64}         = zeros(Int64(MAX_TIME/dt))
ELEVATION::Vector{Float64}      = zeros(Int64(MAX_TIME/dt))
HEADING::Vector{Float64}       = zeros(Int64(MAX_TIME/dt))
HEADING_RATE::Vector{Float64}  = zeros(Int64(MAX_TIME/dt))
BODY_RATE::Vector{Float64}     = zeros(Int64(MAX_TIME/dt))
SET_STEERING::Vector{Float64}  = zeros(Int64(MAX_TIME/dt))
STEERING::Vector{Float64}      = zeros(Int64(MAX_TIME/dt))

function sim_parking(integrator)
    upwind_dir=UPWIND_DIR
    start_time_ns = time_ns()
    clear_viewer(viewer)
    i=1; j=0; k=0
    GC.gc()
    max_time = 0
    t_gc_tot = 0
    sys_state = SysState(kps4)
    on_new_systate(ssc, sys_state)
    while true
        time = i * dt 
        steering = 0.0
        if i > 100
            elevation = sys_state.elevation
            chi_set = pcm.navigate(pc, sys_state.azimuth, elevation)
            steering, _, _, _ = pcm.calc_steering(pc, sys_state.heading, sys_state.heading_rate, chi_set; 
                                                                         elevation, v_app = sys_state.v_app)
            set_depower_steering(kps4.kcu, MIN_DEPOWER, steering)
        end  
        SET_STEERING[i] = steering
        STEERING[i] = get_steering(kps4.kcu) / set.cs_4p
        # execute winch controller
        v_ro = 0.0
        if time > 20
            upwind_dir += deg2rad(0.04*2)
            if upwind_dir > UPWIND_DIR2
                upwind_dir = UPWIND_DIR2
            end
            UPWIND_DIR_[i] = upwind_dir
            # av_upwind_dir = moving_average(UPWIND_DIR_[1:i], 400)
            av_upwind_dir = upwind_dir
        else
            upwind_dir=UPWIND_DIR
            UPWIND_DIR_[i] = upwind_dir
            av_upwind_dir = upwind_dir
        end
        t_sim = @elapsed KiteModels.next_step!(kps4, integrator; set_speed=v_ro, dt, upwind_dir=av_upwind_dir)
        AV_UPWIND_DIR[i] = av_upwind_dir
        V_WIND_KITE[i] = norm(v_wind_kite(kps4))
        if t_sim < 0.3*dt
            t_gc_tot += @elapsed GC.gc(false)
        end
        KiteModels.update_sys_state!(sys_state, kps4)
        T[i] = dt * i
        AZIMUTH[i] = sys_state.azimuth
        AZIMUTH_EAST[i] = calc_azimuth_east(kps4)
        ELEVATION[i] = sys_state.elevation
        FORCE[i] = sys_state.winch_force[1]
        HEADING[i] = wrap2pi(sys_state.heading)
        HEADING_RATE[i] = sys_state.heading_rate
        BODY_RATE[i] = sys_state.turn_rates[3]
        on_new_systate(ssc, sys_state)
        if mod(i, TIME_LAPSE_RATIO) == 0
            if KiteUtils.PROJECT == "system.yaml"
                KiteViewers.update_system(viewer, sys_state; scale = 0.08, kite_scale=3)
            else
                KiteViewers.update_system(viewer, sys_state; scale = 0.08*0.5, kite_scale=3)
            end
            set_status(viewer, String(Symbol(ssc.state)))
            if CREATE_VIDEO
                save_png(viewer, index=div(i, TIME_LAPSE_RATIO))
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

function play_parking()
    saved_use_turbulence = set.use_turbulence
    set.use_turbulence = 0
    integrator = KiteModels.init!(kps4; delta=0.001, stiffness_factor=0.01)
    default_turbulence = get_default_turbulence()
    set.use_turbulence = isnothing(default_turbulence) ? saved_use_turbulence : default_turbulence
    toc()
    try
        sim_parking(integrator)
    catch e
        if isa(e, AssertionError)
            println("AssertionError! Halting simulation.")
        else
            println("Exception! Halting simulation.")
            throw(e) 
        end
    end
    GC.enable(true)
end

play_parking()
stop(viewer)
if CREATE_VIDEO
    using FFMPEG_jll
    FFMPEG_jll.ffmpeg() do exe
        run(`$exe -y -r:v 20 -i video/video%06d.png -codec:v libx264 -preset veryslow -pix_fmt yuv420p -crf 10 -an output/parking_wind_dir.mp4`)
    end
    println("Video saved as output/parking_wind_dir.mp4")
end
let v = filter(!=(0.0), V_WIND_KITE)
    ti = std(v) / mean(v) * 100
    println("Turbulence intensity (wind speed magnitude): $(round(ti, digits=2)) %")
    global p=plotx(T, rad2deg.(AZIMUTH), rad2deg.(AZIMUTH_EAST),[rad2deg.(UPWIND_DIR_), rad2deg.(AV_UPWIND_DIR)],
             rad2deg.(ELEVATION), rad2deg.(HEADING), [100*(SET_STEERING), 100*(STEERING)], V_WIND_KITE, FORCE; 
             xlabel="Time [s]", 
             ysize=10,
             ylabels=["Azimuth [°]", "azimuth_east [°]", "upwind_dir [°]", "Elevation [°]", "Heading [°]", "Steering [%]", "v_wind_kite [m/s]", "force [N]"],
             labels=["azimuth", "azimuth_east", ["upwind_dir", "filtered_upwind_dir"], "elevation", "heading", ["set_steering", "steering"], "v_wind_kite", "force"],
             fig="Parking with changing wind direction, TI: $(round(ti, digits=2)) %")
end
display(p)
if PLOT_RATES
    p2 = plot(T, [rad2deg.(HEADING_RATE), rad2deg.(BODY_RATE)];
               xlabel = "time [s]", ylabel = "rate [°/s]",
               labels = ["heading_rate", "body_rate"], fig = "rates")
    display(p2)
end
reactivate_host_app()
