using Pkg
if ! ("KiteViewers" ∈ keys(Pkg.project().dependencies))
    using TestEnv; TestEnv.activate()
end
using Timers; tic()

using KiteControllers, KiteViewers, KiteModels, StatsBase


set = deepcopy(load_settings("system.yaml"))
set.segments = 12
kcu::KCU   = KCU(set)
kps4::KPS4 = KPS4(kcu)

wcs::WCSettings = WCSettings(true, dt = 1/set.sample_freq)
fcs::FPCSettings = FPCSettings(true, dt=wcs.dt)
fpps::FPPSettings = FPPSettings(true)
u_d0 = 0.01 * set.depower_offset
u_d  = 0.01 * set.depower
ssc::SystemStateControl = SystemStateControl(wcs, fcs, fpps; u_d0, u_d, v_wind=set.v_wind)
dt::Float64 = wcs.dt

function init_globals()
    global kcu, kps4, wcs, fcs, fpps, ssc
    kcu   = KCU(set)
    kps4 = KPS4(kcu)
    wcs  = WCSettings(true, dt = 1/set.sample_freq)
    fcs  = FPCSettings(true, dt=wcs.dt)
    fpps = FPPSettings(true)
    u_d0 = 0.01 * set.depower_offset
    u_d  = 0.01 * set.depower
    ssc = SystemStateControl(wcs, fcs, fpps; u_d0, u_d, v_wind=set.v_wind)
end

# the following values can be changed to match your interest
MAX_TIME::Float64 = 460
TIME_LAPSE_RATIO  = 4
SHOW_KITE         = true
# end of user parameter section #

phi_set = 21.48
# on_control_command(ssc.fpp.fpca.fpc, attractor=[deg2rad(phi_set), deg2rad(51.88)])
# on_control_command(ssc.fpp.fpca.fpc, psi_dot_set=-23.763, radius=-4.35)

viewer::Viewer3D = Viewer3D(set, SHOW_KITE)
PARKING::Bool = false

steps = 0
T::Vector{Float64} = zeros(Int64(MAX_TIME/dt))
DELTA_T::Vector{Float64} = zeros(Int64(MAX_TIME/dt))
STEERING::Vector{Float64}      = zeros(Int64(MAX_TIME/dt))
DEPOWER_::Vector{Float64}      = zeros(Int64(MAX_TIME/dt))
LAST_I::Int64=0

function simulate(integrator, stopped=true)
    global LAST_I
    start_time_ns = time_ns()
    clear_viewer(viewer)
    KiteViewers.running[] = ! stopped
    viewer.stop = stopped
    if ! stopped
        set_status(viewer, "ssParking")
    end
    i=1
    j=0; k=0
    GC.gc()
    if Sys.total_memory()/1e9 > 24 && MAX_TIME < 500
        GC.enable(false)
    end
    t_gc_tot = 0
    sys_state = SysState(kps4)
    on_new_systate(ssc, sys_state)
    KiteViewers.update_system(viewer, sys_state; scale = 0.04/1.1, kite_scale=6.6)
    while true
        if viewer.stop
            sleep(dt)
        else
            if i > 100
                dp = KiteControllers.get_depower(ssc)
                if dp < 0.22 dp = 0.22 end
                heading = calc_heading(kps4; neg_azimuth=true, one_point=false)
                ssc.sys_state.heading = heading
                ssc.sys_state.azimuth = -calc_azimuth(kps4)
                steering = -calc_steering(ssc)
                set_depower_steering(kps4.kcu, dp, steering)
            end
            if i == 200 && ! PARKING
                on_autopilot(ssc)
            end
            # execute winch controller
            v_ro = calc_v_set(ssc)
            #
            t_sim = @elapsed KiteModels.next_step!(kps4, integrator; set_speed=v_ro, dt=dt)
            sys_state = SysState(kps4)
            if i <= length(T)
                T[i] = dt * i
                if i > 10/dt
                    DELTA_T[i]  = t_sim * 1000
                    STEERING[i] = sys_state.steering
                    DEPOWER_[i] = sys_state.depower
                    LAST_I=i
                end
            end
            on_new_systate(ssc, sys_state)
            if mod(i, TIME_LAPSE_RATIO) == 0 
                KiteViewers.update_system(viewer, sys_state; scale = 0.04/1.1, kite_scale=6.6)
                set_status(viewer, String(Symbol(ssc.state)))
                # turn garbage collection back on if we are short of memory
                if Sys.free_memory()/1e9 < 2.0
                    GC.enable(true)
                end
                wait_until(start_time_ns + 1e9*dt, always_sleep=true)          
                start_time_ns = time_ns()
                t_gc_tot = 0
            end
            i += 1
        end
        if ! isopen(viewer.fig.scene) break end
        if KiteViewers.status[] == "Stopped" && i > 10 break end
        if i*dt > MAX_TIME break end
    end
    return div(i, TIME_LAPSE_RATIO)
end

function play(stopped=false)
    global steps, kcu, kps4, wcs, fcs, fpps, ssc
    init_globals()
    on_parking(ssc)
    integrator = KiteModels.init!(kps4, stiffness_factor=0.04)
    toc()
    steps = simulate(integrator, stopped)
    stopped = ! viewer.sw.active[]
    GC.enable(true)
end

function parking()
    global PARKING
    PARKING = true
    viewer.stop=false
    on_parking(ssc)
end

function autopilot()
    global PARKING
    PARKING = false
    viewer.stop=false
    on_autopilot(ssc)
end

function stop_()
    println("Stopping...")
    on_stop(ssc)
    clear!(kps4)
    clear_viewer(viewer)
end

stop_()
on(viewer.btn_PARKING.clicks) do c; parking(); end
on(viewer.btn_AUTO.clicks) do c; autopilot(); end
on(viewer.btn_STOP.clicks) do c; stop_(); end
on(viewer.btn_PLAY.clicks) do c;
    global PARKING
    if ! viewer.stop
        PARKING = false
    end
end

play(false)
stop_()
KiteViewers.GLMakie.closeall()

GC.enable(true)
