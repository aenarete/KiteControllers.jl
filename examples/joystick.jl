# activate the test environment if needed
using Pkg
if ! ("KiteViewers" ∈ keys(Pkg.project().dependencies))
    using TestEnv; TestEnv.activate()
end
using Timers; tic()

using KiteControllers, KiteViewers, KiteModels, Joysticks

set = deepcopy(load_settings("system.yaml"))
kcu::KCU = KCU(set)
kps4::KPS4 = KPS4(kcu)
if ! @isdefined js;
    const js = open_joystick();
    const jsaxes = JSState(); 
    const jsbuttons = JSButtonState()
    async_read!(js, jsaxes, jsbuttons)
end
wcs::WCSettings = WCSettings(true, dt = 1/set.sample_freq)
fcs::FPCSettings =  FPCSettings(true, dt=wcs.dt)
fpps::FPPSettings = FPPSettings(true)
u_d0 = 0.01 * set.depower_offset
u_d  = 0.01 * set.depower
ssc::SystemStateControl = SystemStateControl(wcs, fcs, fpps; u_d0, u_d, v_wind = set.v_wind)
dt::Float64 = wcs.dt

# the following values can be changed to match your interest
MAX_TIME::Float64 = 3600
TIME_LAPSE_RATIO  = 1
SHOW_KITE         = true
# end of user parameter section #

viewer::Viewer3D = Viewer3D(SHOW_KITE, "WinchON")

steps = 0

function simulate(integrator)
    start_time_ns = time_ns()
    clear_viewer(viewer)
    viewer.stop = false
    i=1; j=0; k=0
    GC.gc()
    max_time = 0
    t_gc_tot = 0
    e_mech = 0.0
    sys_state = SysState(kps4)
    on_new_systate(ssc, sys_state)
    while true
        if i > 100
            # depower = 0.22 - jsaxes.y*0.4
            depower = KiteControllers.get_depower(ssc)
            # println("dp: ", dp)
            if depower < 0.22; depower = 0.22; end
            heading = calc_heading(app.kps4; neg_azimuth=true)
            steering = calc_steering(ssc, jsaxes.x; heading)
            set_depower_steering(kps4.kcu, depower, steering)
            println("depower: ", depower, " steering: ", round(steering, digits=3))
            # set_depower_steering(kps4.kcu, depower, jsaxes.x)
            # v_ro = jsaxes.u * 8.0 
        end  
        # execute winch controller
        v_ro = calc_v_set(ssc)
        t_sim = @elapsed KiteModels.next_step!(kps4, integrator; set_speed=v_ro, dt=dt)
        if t_sim < 0.3*dt
            t_gc_tot += @elapsed GC.gc(false)
        end
        sys_state = SysState(kps4)
        on_new_systate(ssc, sys_state)
        e_mech += (sys_state.force * sys_state.v_reelout)/3600*dt
        sys_state.e_mech = e_mech
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
        if i*dt > MAX_TIME break end
        i += 1
    end
    misses = j/k * 100
    println("\nMissed the deadline for $(round(misses, digits=2)) %. Max time: $(round((max_time*1e-6), digits=1)) ms")
    return div(i, TIME_LAPSE_RATIO)
end

function play()
    global steps
    integrator = KiteModels.init!(kps4, stiffness_factor=0.04)
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
    on_winchcontrol(ssc)
end

on(viewer.btn_PLAY.clicks) do c; async_play(); end
on(viewer.btn_STOP.clicks) do c; stop(viewer); on_stop(ssc) end
on(viewer.btn_PARKING.clicks) do c; parking(); end
on(viewer.btn_AUTO.clicks) do c; autopilot(); end

on(jsbuttons.btn1) do val; if val async_play() end; end
on(jsbuttons.btn2) do val; if val stop(viewer) end; end
on(jsbuttons.btn3) do val; if val autopilot() end; end
on(jsbuttons.btn4) do val; if val on_reelin(ssc) end; end
on(jsbuttons.btn5) do val; if val on_parking(ssc) end; end

play()
stop(viewer)
