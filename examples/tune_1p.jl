# activate the test environment if needed
using Pkg
if ! ("Plots" ∈ keys(Pkg.project().dependencies))
    using TestEnv; TestEnv.activate()
end

using KiteUtils
se().abs_tol=0.0000006
se().rel_tol=0.000001

using KiteControllers, KiteModels, Plots

if ! @isdefined kcu;   const kcu = KCU(se());   end
if ! @isdefined kps;   const kps = KPS3(kcu); end
wcs = WCSettings(); wcs.dt = 1/se().sample_freq
fcs = FPCSettings(); fcs.dt = wcs.dt
fpps = FPPSettings()
const ssc = SystemStateControl(wcs, fcs, fpps)
dt = wcs.dt

# the following values can be changed to match your interest
if ! @isdefined MAX_TIME; MAX_TIME=60; end
TIME_LAPSE_RATIO = 1
SHOW_KITE = false
# end of user parameter section #

steps = 0
if ! @isdefined T;       const T = zeros(Int64(MAX_TIME/dt)); end
if ! @isdefined AZIMUTH; const AZIMUTH = zeros(Int64(MAX_TIME/dt)); end

function simulate(integrator)
    i=1
    sys_state = SysState(kps)
    on_new_systate(ssc, sys_state)
    while true
        if i > 100
            depower = KiteControllers.get_depower(ssc)
            if depower < 0.22; depower = 0.22; end
            steering = calc_steering(ssc, 0)
            time = i * dt
            # disturbance
            if time > 20 && time < 21
                steering = 0.1
            end
            set_depower_steering(kps.kcu, depower, steering)
        end  
        v_ro = 0.0
        KiteModels.next_step!(kps, integrator, v_ro=v_ro, dt=dt)
        sys_state = SysState(kps)
        T[i] = dt * i
        AZIMUTH[i] = sys_state.azimuth        
        on_new_systate(ssc, sys_state)
        if i*dt >= MAX_TIME break end
        i += 1
    end
    return 1
end

# tests the parking controller and returns the sum of the square of 
# the azimuth error in degrees squared and divided by the test duration
function test_parking(show_plot=false)
    clear!(kps)
    init_kcu(kcu, se())
    integrator = KiteModels.init_sim!(kps, stiffness_factor=0.04)
    simulate(integrator)
    if show_plot
        plot(T, rad2deg.(AZIMUTH))
    end
    sum(abs2.(rad2deg.(AZIMUTH)))/40
end

test_parking()
