# park the kind while the wind direction changes
using Pkg
if ! ("ControlPlots" ∈ keys(Pkg.project().dependencies))
    using Pkg
    using TestEnv; TestEnv.activate()
end
using Timers; tic()

using KiteControllers, KiteViewers, KiteModels, ControlPlots, Rotations, StatsBase

set = deepcopy(load_settings("system.yaml"))
@assert KiteUtils.PROJECT == "system.yaml"
@assert se().v_wind == 9.51
@assert set.v_wind == 9.51
set.abs_tol=0.00006
set.rel_tol=0.0001

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
u_d = 0.01 * set.depower
ssc::SystemStateControl = SystemStateControl(wcs, fcs, fpps; u_d0, u_d, v_wind = set.v_wind)
dt::Float64 = wcs.dt

# result of tuning
fcs.p=1.3
fcs.i=0.2
fcs.d=13.25*0.9
fcs.use_chi = false
@assert fcs.gain == 0.04

# the following values can be changed to match your interest
MAX_TIME::Float64 = 160
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
HEADING::Vector{Float64}       = zeros(Int64(MAX_TIME/dt))
SET_STEERING::Vector{Float64}  = zeros(Int64(MAX_TIME/dt))
STEERING::Vector{Float64}      = zeros(Int64(MAX_TIME/dt))

function sim_parking(integrator)
    upwind_dir=UPWIND_DIR
    av_upwind_dir = upwind_dir
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
            depower = KiteControllers.get_depower(ssc)
            if depower < 0.22; depower = 0.22; end
            heading = calc_heading(kps4; neg_azimuth=true, one_point=false)
            steering = calc_steering(ssc, 0; heading)
           
            set_depower_steering(kps4.kcu, depower, steering)
        end  
        SET_STEERING[i] = steering
        STEERING[i] = get_steering(kps4.kcu)
        # execute winch controller
        v_ro = 0.0
        if time > 20
            upwind_dir += deg2rad(0.04*2)
            if upwind_dir > UPWIND_DIR2
                upwind_dir = UPWIND_DIR2
            end
            UPWIND_DIR_[i] = upwind_dir
            av_upwind_dir = moving_average(UPWIND_DIR_[1:i], 400)
        else
            upwind_dir=UPWIND_DIR
            UPWIND_DIR_[i] = upwind_dir
            av_upwind_dir = upwind_dir
        end
        t_sim = @elapsed KiteModels.next_step!(kps4, integrator; set_speed=v_ro, dt, upwind_dir=av_upwind_dir)
        AV_UPWIND_DIR[i] = av_upwind_dir
        if t_sim < 0.3*dt
            t_gc_tot += @elapsed GC.gc(false)
        end
        sys_state = SysState(kps4)
        sys_state.orient .= calc_orient_quat(kps4)
        T[i] = dt * i
        AZIMUTH[i] = sys_state.azimuth
        AZIMUTH_EAST[i] = calc_azimuth_east(kps4)
        HEADING[i] = wrap2pi(sys_state.heading)
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

function play_parking()
    integrator = KiteModels.init_sim!(kps4; delta=0.001, stiffness_factor=0.5)
    toc()
    try
        steps = sim_parking(integrator)
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
p=plotx(T, rad2deg.(AZIMUTH), rad2deg.(AZIMUTH_EAST),[rad2deg.(UPWIND_DIR_), rad2deg.(AV_UPWIND_DIR)],
         rad2deg.(HEADING), [100*(SET_STEERING), 100*(STEERING)]; 
         xlabel="Time [s]", 
         ylabels=["Azimuth [°]", "azimuth_east [°]", "upwind_dir [°]", "Heading [°]", "Steering [%]"],
         labels=["azimuth", "azimuth_east", ["upwind_dir", "filtered_upwind_dir"], "heading", ["set_steering", "steering"]])
display(p)
