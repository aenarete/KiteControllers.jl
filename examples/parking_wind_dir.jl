# park the kind while the wind direction changes
using Pkg
if ! ("ControlPlots" ∈ keys(Pkg.project().dependencies))
    using Pkg
    pkg"add KiteModels#azimuth"
    using TestEnv; TestEnv.activate()
end
using Timers; tic()

# using Pkg
# pkg"add KiteModels#azimuth"

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
fcs.p=1.3 #1.5
fcs.i=0.2
fcs.d=13.25
fcs.use_chi = false

# the following values can be changed to match your interest
MAX_TIME::Float64 = 100
TIME_LAPSE_RATIO  =  4
SHOW_KITE         = true
# For position and velocity vectors of the model the ENU (East North Up) 
UPWIND_DIR        = -pi/2 # the direction the wind is coming from.
UPWIND_DIR2       = -pi/2+deg2rad(20)     # Zero is at north; clockwise positive
# end of user parameter section #

viewer::Viewer3D = Viewer3D(SHOW_KITE, "WinchON")

steps = 0
T::Vector{Float64} = zeros(Int64(MAX_TIME/dt))
if ! @isdefined AZIMUTH; const AZIMUTH = zeros(Int64(MAX_TIME/dt)); end
if ! @isdefined AZIMUTH_EAST; const AZIMUTH_EAST = zeros(Int64(MAX_TIME/dt)); end
if ! @isdefined UPWIND_DIR_; const UPWIND_DIR_ = zeros(Int64(MAX_TIME/dt)); end
if ! @isdefined HEADING; const HEADING = zeros(Int64(MAX_TIME/dt)); end
if ! @isdefined STEERING; const SET_STEERING = zeros(Int64(MAX_TIME/dt)); end
if ! @isdefined STEERING; const STEERING = zeros(Int64(MAX_TIME/dt)); end
# """
#     calc_heading_w(orientation, down_wind_direction = pi/2.0)

# Calculate the heading vector in wind reference frame.
# """
# function calc_heading_w(orientation, down_wind_direction = pi/2.0)
#     # create a unit heading vector in the xsense reference frame
#     heading_sensor =  SVector(1, 0, 0)
#     # rotate headingSensor to the Earth Xsens reference frame
#     headingEX = fromKS2EX(heading_sensor, orientation)
#     # rotate headingEX to earth groundstation reference frame
#     headingEG = fromEX2EG(headingEX)
#     # rotate headingEG to headingW and convert to 2d HeadingW vector
#     fromEG2W(headingEG, down_wind_direction)
# end

# """
#     calc_heading(orientation, elevation, azimuth; upwind_dir=-pi/2, respos=true)

# Calculate the heading angle of the kite in radians. The heading is the direction
# the nose of the kite is pointing to. 
# If respos is true the heading angle is defined in the range of 0 .. 2π,
# otherwise in the range -π .. π
# """
# function calc_heading(orientation, elevation, azimuth; upwind_dir=-pi/2, respos=true)
#     down_wind_direction = wrap2pi(upwind_dir + π)
#     headingSE = fromW2SE(calc_heading_w(orientation, down_wind_direction), elevation, azimuth)
#     angle = atan(headingSE.y, headingSE.x) # - π
#     if angle < 0 && respos
#         angle += 2π
#     end
#     angle
# end


# #  function calc_heading(s::KPS4; upwind_dir=upwind_dir(s))
# function calc_heading(s::KPS4; upwind_dir=-pi/2)
#     orientation = orient_euler(s)
#     elevation = calc_elevation(s)
#     azimuth = calc_azimuth(s)
#     KiteUtils.calc_heading(orientation, elevation, azimuth; upwind_dir)
# end

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
        steering = 0.0
        if i > 100
            depower = KiteControllers.get_depower(ssc)
            if depower < 0.22; depower = 0.22; end
            steering = -calc_steering(ssc, 0)
           
            set_depower_steering(kps4.kcu, depower, steering)
        end  
        SET_STEERING[i] = steering
        STEERING[i] = get_steering(kps4.kcu)
        # execute winch controller
        v_ro = 0.0
        if time > 20 && upwind_dir < UPWIND_DIR2
            upwind_dir += deg2rad(0.05)
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
        sys_state.orient .= calc_orient_quat(kps4)
        T[i] = dt * i
        AZIMUTH[i] = sys_state.azimuth
        AZIMUTH_EAST[i] = calc_azimuth_east(kps4)
        # HEADING[i] = wrap2pi(calc_heading(kps4)) 
        HEADING[i] = wrap2pi(sys_state.heading)
        on_new_systate(ssc, sys_state)
        if mod(i, TIME_LAPSE_RATIO) == 0
            sys_state.orient .= quat2viewer(calc_orient_quat(kps4))
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
            throw(e) 
        end
    end
    GC.enable(true)
end

play()
stop(viewer)
plotx(T, rad2deg.(AZIMUTH), rad2deg.(AZIMUTH_EAST),rad2deg.(UPWIND_DIR_), rad2deg.(HEADING), [100*(SET_STEERING), 100*(STEERING)]; 
         xlabel="Time [s]", 
         ylabels=["Azimuth [°]", "azimuth_east [°]", "upwind_dir [°]", "Heading [°]", "Steering [°]"],
         labels=["azimuth", "azimuth_east", "upwind_dir", "heading", ["set_steering", "steering"]])
