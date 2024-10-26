# activate the test environment if needed
using Pkg
if ! ("ControlPlots" ∈ keys(Pkg.project().dependencies))
    using TestEnv; TestEnv.activate()
end
using Timers; tic()

using KiteControllers, KiteViewers, KiteModels, Rotations

set = deepcopy(load_settings("system.yaml"))

kcu::KCU = KCU(set)
kps3::KPS3 = KPS3(kcu)

wcs::WCSettings = WCSettings(dt = 1/set.sample_freq)
fcs::FPCSettings = FPCSettings(dt = wcs.dt)
fpps::FPPSettings = FPPSettings()
u_d0 = 0.01 * set.depower_offset
u_d = 0.01 * set.depower
ssc::SystemStateControl = SystemStateControl(wcs, fcs, fpps; u_d0, u_d, v_wind=set.v_wind)
dt::Float64 = wcs.dt

# result of tuning, factor 0.9 to increase robustness
fcs.p = 13.63*0.9
fcs.i = 0.0
fcs.d = 27.75*0.9

# the following values can be changed to match your interest
MAX_TIME::Float64=3600
TIME_LAPSE_RATIO = 2
SHOW_KITE = true
# end of user parameter section #

phi_set = 21.48
# on_control_command(ssc.fpp.fpca.fpc, attractor=[deg2rad(phi_set), deg2rad(51.88)])
# on_control_command(ssc.fpp.fpca.fpc, psi_dot_set=-23.763, radius=-4.35)

viewer::Viewer3D = Viewer3D(SHOW_KITE)

steps = 0

function simulate(integrator)
    start_time_ns = time_ns()
    clear_viewer(viewer)
    i=1
    j=0; k=0
    GC.gc()
    max_time = 0
    t_gc_tot = 0
    sys_state = SysState(kps3)
    on_new_systate(ssc, sys_state)
    while true
        if i > 100
            dp = KiteControllers.get_depower(ssc)
            if dp < 0.22 dp = 0.22 end
            heading = calc_heading(kps3; neg_azimuth=true)
            steering = calc_steering(ssc, 0; heading)
            set_depower_steering(kps3.kcu, dp, steering)
        end
        if i == 200
            on_autopilot(ssc)
        end
        # execute winch controller
        v_ro = calc_v_set(ssc)
        #
        t_sim = @elapsed KiteModels.next_step!(kps3, integrator; set_speed=v_ro, dt=dt)
        sys_state = SysState(kps3)
        on_new_systate(ssc, sys_state)
        if mod(i, TIME_LAPSE_RATIO) == 0
            sys_state.orient = quat2viewer(QuatRotation(sys_state.orient))
            KiteViewers.update_system(viewer, sys_state; scale = 0.04/1.1, kite_scale=6.6)
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
        if i*dt > MAX_TIME break end
        i += 1
    end
    misses = j/k * 100
    println("\nMissed the deadline for $(round(misses, digits=2)) %. Max time: $(round((max_time*1e-6), digits=1)) ms")
    return div(i, TIME_LAPSE_RATIO)
end

function play()
    global steps
    integrator = KiteModels.init_sim!(kps3, stiffness_factor=0.04)
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
    on_autopilot(ssc)
end

on(viewer.btn_PLAY.clicks) do c; async_play(); end
on(viewer.btn_STOP.clicks) do c; stop(viewer); on_stop(ssc) end
on(viewer.btn_PARKING.clicks) do c; parking(); end
on(viewer.btn_AUTO.clicks) do c; autopilot(); end

play()
stop(viewer)
