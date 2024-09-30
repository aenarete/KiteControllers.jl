# park the kind while the wind direction changes
using Pkg
if ! ("ControlPlots" ∈ keys(Pkg.project().dependencies))
    using TestEnv; TestEnv.activate()
end
using Timers; tic()

using Pkg
pkg"add KiteModels#fix_yaw"

using KiteControllers, KiteViewers, KiteModels, ControlPlots, Rotations
set = deepcopy(load_settings("system.yaml"))
set.abs_tol=0.00006
set.rel_tol=0.0001

kcu::KCU = KCU(set)
kps4::KPS4 = KPS4(kcu)
wcs::WCSettings = WCSettings(dt = 1/set.sample_freq)
fcs::FPCSettings = FPCSettings(dt = wcs.dt)
fpps::FPPSettings = FPPSettings()
u_d0 = 0.01 * set.depower_offset
u_d = 0.01 * set.depower
ssc::SystemStateControl = SystemStateControl(wcs, fcs, fpps; u_d0, u_d)
dt::Float64 = wcs.dt

# result of tuning
fcs.p=2.0 #1.5
fcs.i=0.35
fcs.d=13.25

# the following values can be changed to match your interest
MAX_TIME::Float64 = 80
TIME_LAPSE_RATIO  =  4
SHOW_KITE         = true
# For position and velocity vectors of the model the ENU (East North Up) 
UPWIND_DIR        = -pi/2 # the direction the wind is coming from.
UPWIND_DIR2       = -pi/2+deg2rad(10)     # Zero is at north; clockwise positive
# end of user parameter section #

viewer::Viewer3D = Viewer3D(SHOW_KITE, "WinchON")

steps = 0
T::Vector{Float64} = zeros(Int64(MAX_TIME/dt))
if ! @isdefined AZIMUTH; const AZIMUTH = zeros(Int64(MAX_TIME/dt)); end
if ! @isdefined UPWIND_DIR_; const UPWIND_DIR_ = zeros(Int64(MAX_TIME/dt)); end
if ! @isdefined HEADING; const HEADING = zeros(Int64(MAX_TIME/dt)); end
if ! @isdefined STEERING; const SET_STEERING = zeros(Int64(MAX_TIME/dt)); end
if ! @isdefined STEERING; const STEERING = zeros(Int64(MAX_TIME/dt)); end

function simulate(integrator)
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
        if i > 100
            depower = KiteControllers.get_depower(ssc)
            if depower < 0.22; depower = 0.22; end
            steering = calc_steering(ssc, 0)
           
            set_depower_steering(kps4.kcu, depower, steering)
        end  
        
        # execute winch controller
        v_ro = 0.0
        if time > 20 && upwind_dir < UPWIND_DIR2
            upwind_dir += deg2rad(0.02)
            if upwind_dir > UPWIND_DIR2
                upwind_dir = UPWIND_DIR2
            end
        end
        t_sim = @elapsed KiteModels.next_step!(kps4, integrator; set_speed=v_ro, dt, upwind_dir)
        UPWIND_DIR_[i] = KiteModels.upwind_dir(kps4)
        if t_sim < 0.3*dt
            t_gc_tot += @elapsed GC.gc(false)
        end
        sys_state = SysState(kps4)
        T[i] = dt * i
        AZIMUTH[i] = sys_state.azimuth
        on_new_systate(ssc, sys_state)
        if mod(i, TIME_LAPSE_RATIO) == 0
            q = QuatRotation(sys_state.orient)
            q_viewer = AngleAxis(-π/2, 0, 1, 0) * q
            sys_state.orient .= Rotations.params(q_viewer)
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
    integrator = KiteModels.init_sim!(kps4, stiffness_factor=0.04)
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

play()
stop(viewer)
plotx(T, rad2deg.(AZIMUTH), rad2deg.(UPWIND_DIR_), rad2deg.(HEADING), rad2deg.(STEERING); 
         xlabel="Time [s]", 
         ylabels=["Azimuth [°]", "upwind_dir [°]", "Heading [°]", "Steering [°]"])
