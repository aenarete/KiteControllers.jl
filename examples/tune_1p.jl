# activate the test environment if needed
using Pkg
if ! ("Plots" ∈ keys(Pkg.project().dependencies))
    using TestEnv; TestEnv.activate()
end

using KiteUtils
se().abs_tol=0.0000006
se().rel_tol=0.000001

using KiteControllers, KiteModels, Plots, BayesOpt

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

LAST_RES = 1e10
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
            if time < 20
                steering = 0.0
            elseif time < 21
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
function test_parking(suppress_overshoot_factor = 6.0)
    global LAST_RES
    clear!(kps)
    init_kcu(kcu, se())
    AZIMUTH .= zeros(Int64(MAX_TIME/dt))
    integrator = KiteModels.init_sim!(kps, stiffness_factor=0.04)
    simulate(integrator)
    res = sum(abs2.(rad2deg.(AZIMUTH)))/40
    n = length(AZIMUTH)
    overshoot = zeros(n)
    for i in 1:n
        if AZIMUTH[i] < 0
            overshoot[i] = -AZIMUTH[1]
        end
    end
    res += sum(abs2.(rad2deg.(overshoot)))/40 * suppress_overshoot_factor
    if res < LAST_RES
        LAST_RES = res
        println(res, " p: ", fcs.p, " d: ", fcs.d)
        display(show_result())
    end
    res
end

function show_result()
    plot(T, rad2deg.(AZIMUTH))
end

function f(x)
    fcs.p = x[1]
    fcs.i = x[2]
    fcs.d = x[3]
    test_parking()
end

function tune_1p(robustness_factor=0.95)
    config = ConfigParameters()         # calls initialize_parameters_to_default of the C API
    config.noise = 1e-6
    config.n_iterations = 60
    println(config.noise)
    println(config.n_inner_iterations)
    set_kernel!(config, "kMaternARD5")  # calls set_kernel of the C API
    config.sc_type = SC_MAP
    lowerbound = [10., 0., 10.]; upperbound = [30., 1.0, 50.]
    x, optimum = bayes_optimization(f, lowerbound, upperbound, config)
    fcs.p = x[1]*robustness_factor
    fcs.i = x[2]*robustness_factor
    fcs.d = x[3]*robustness_factor
    test_parking()
    show_result()
end

function est_noise(n=10)
    res = zeros(n)
    for i in 1:n
        res[i] = test_parking()
    end
    # println(res)
    avg = sum(res)/n
    noise = res .- avg
    rel_noise = sum(abs2.(noise) / abs2(avg))/n
end

fcs.p=13.63*0.9
fcs.i=0.0
fcs.d=27.75*0.9
println(test_parking())
show_result()
