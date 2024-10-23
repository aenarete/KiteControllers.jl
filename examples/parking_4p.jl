# activate the test environment if needed
using Pkg
if ! ("ControlPlots" ∈ keys(Pkg.project().dependencies))
    using TestEnv; TestEnv.activate()
end
using Timers; tic()

using Pkg
# Pkg.update()
# pkg"add KitePodModels#main"
# pkg"add KiteModels#main"

using KiteControllers, KiteViewers, KiteModels, ControlPlots, Rotations
set = deepcopy(load_settings("system.yaml"))
set.abs_tol=0.00006
set.rel_tol=0.0001
#set.l_tether = 150

kcu::KCU = KCU(set)
kps4::KPS4 = KPS4(kcu)
@assert set.sample_freq == 20
wcs::WCSettings = WCSettings(dt = 1/set.sample_freq)
update(wcs); wcs.dt = 1/set.sample_freq
fcs::FPCSettings = FPCSettings(dt = wcs.dt)
update(fcs); fcs.dt = wcs.dt
fpps::FPPSettings = FPPSettings()
update(fpps)
u_d0 = 0.01 * set.depower_offset
u_d = 0.01 * set.depower
ssc::SystemStateControl = SystemStateControl(wcs, fcs, fpps; u_d0, u_d, v_wind = set.v_wind)
dt::Float64 = wcs.dt

# # result of tuning
# fcs.p=0.60*20
# fcs.i=0.15
# fcs.d=12.34

if KiteUtils.PROJECT == "system.yaml"
    # result of tuning
    fcs.p=1.3
    fcs.i=0.2
    fcs.d=13.25*0.9
    fcs.use_chi = false
    @assert fcs.gain == 0.04
else
    # result of tuning
    println("not system.yaml")
    fcs.p=1.3
    fcs.i=0.1
    fcs.d=13.25*0.9
    fcs.use_chi = false
    fcs.gain = 0.04*0.5
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
if ! @isdefined AZIMUTH; const AZIMUTH = zeros(Int64(MAX_TIME/dt)); end
if ! @isdefined HEADING; const HEADING = zeros(Int64(MAX_TIME/dt)); end
if ! @isdefined STEERING; const SET_STEERING = zeros(Int64(MAX_TIME/dt)); end
if ! @isdefined STEERING; const STEERING = zeros(Int64(MAX_TIME/dt)); end

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
            depower = KiteControllers.get_depower(ssc)
            if depower < 0.22; depower = 0.22; end
            heading = calc_heading(kps4; neg_azimuth=true, one_point=false)
            steering = calc_steering(ssc, 0; heading)
            # steering = 0.15*sys_state.azimuth
            time = i * dt
            # disturbance
            if time > 20 && time < 21
                steering = 0.1
            end            
            set_depower_steering(kps4.kcu, depower, steering)
        end
        SET_STEERING[i] = steering
        STEERING[i] = get_steering(kps4.kcu)
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
            # q = QuatRotation(sys_state.orient)
            # q_viewer = AngleAxis(-π/2, 0, 1, 0) * q
            # sys_state.orient .= Rotations.params(q_viewer)
            # sys_state.orient .= calc_orient_quat(kps4)
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
    integrator = KiteModels.init_sim!(kps4, stiffness_factor=0.04)
    toc()
    # try
        steps = simulate(integrator)
    # catch e
    #     if isa(e, AssertionError)
    #         println("AssertionError! Halting simulation.")
    #     else
    #         println("Exception! Halting simulation.")
    #     end
    # end
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
p = plotx(T, rad2deg.(AZIMUTH), rad2deg.(HEADING), [100*(SET_STEERING), 100*(STEERING)]; 
          xlabel="Time [s]", 
          ylabels=["Azimuth [°]", "Heading [°]", "steering [%]"],
          labels=["azimuth", "heading", ["set_steering", "steering"]], 
          fig="Azimuth and Heading")
display(p)
