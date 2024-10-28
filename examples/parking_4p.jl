# activate the test environment if needed
using Pkg
if ! ("ControlPlots" ∈ keys(Pkg.project().dependencies))
    using TestEnv; TestEnv.activate()
end
using Timers; tic()

using KiteControllers, KiteViewers, KiteModels, ControlPlots, Rotations
set = deepcopy(load_settings("system_v9.yaml"))
set.abs_tol=0.00006
set.rel_tol=0.0001
set.v_wind = 5.5 # v_min1 6-25; v_min2 5.5-25

include("parking_controller.jl")
pcs = ParkingControllerSettings(dt=0.05)

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
    pcs.kp_tr=0.06
    pcs.ki_tr=0.0012
    pcs.kp = 15
    pcs.ki = 0.5
    MIN_DEPOWER       = 0.22
    DISTURBANCE      = 0.1
else
    # result of tuning
    println("not system.yaml")
    pcs.kp_tr=0.05
    pcs.ki_tr=0.0024
    pcs.kp = 30
    pcs.ki = 1.0
    MIN_DEPOWER       = 0.4
    DISTURBANCE      = 0.4
    pcs.c1 = 0.048
    pcs.c2 = 0#5.5 # has no big effect, can also be set to zero
end
println("pcs.kp_tr=$(pcs.kp_tr), pcs.ki_tr=$(pcs.ki_tr), pcs.kp=$(pcs.kp), pcs.ki=$(pcs.ki), MIN_DEPOWER=$(MIN_DEPOWER)")
pc = ParkingController(pcs)

# the following values can be changed to match your interest
MAX_TIME::Float64 = 120 # was 60
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
PSI_DOT::Vector{Float64}       = zeros(Int64(MAX_TIME/dt))
PSI_DOT_SET::Vector{Float64}   = zeros(Int64(MAX_TIME/dt))

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
        if i >= 100
            heading = calc_heading(kps4; neg_azimuth=true, one_point=false)
            if i == 100
                pc.last_heading = heading
            end
            elevation = sys_state.elevation
            # println("heading: $(rad2deg(heading)), elevation: $(rad2deg(elevation))")
            chi_set = -navigate(pc, sys_state.azimuth, elevation)
            u_d, ndi_gain, psi_dot, psi_dot_set = calc_steering(pc, heading, chi_set; elevation, v_app = sys_state.v_app)
            steering = calc_steering(ssc, 0; heading)
            # println("chi_set: $(rad2deg(chi_set)), heading: $(rad2deg(wrap2pi(heading)))")
            # println("steering, u_d: $(steering), $(u_d)")
            # println("psi_dot, psi_dot_set, ndi_gain: $(rad2deg(psi_dot)), $(rad2deg(psi_dot_set)), $(ndi_gain)")
            PSI_DOT[i] = psi_dot
            PSI_DOT_SET[i] = psi_dot_set
            steering = u_d
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
p = plotx(T, rad2deg.(AZIMUTH), rad2deg.(HEADING), [100*(SET_STEERING), 100*(STEERING)], [rad2deg.(PSI_DOT), rad2deg.(PSI_DOT_SET)]; 
          xlabel="Time [s]", 
          ylabels=["Azimuth [°]", "Heading [°]", "steering [%]", "psi_dot [°/s]"],
          labels=["azimuth", "heading", ["set_steering", "steering"], ["psi_dot", "psi_dot_set"]], 
          fig="Azimuth, heading, steering and AoA",)
display(p)
