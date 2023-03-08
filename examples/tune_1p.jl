# activate the test environment if needed
using Pkg
if ! ("Plots" ∈ keys(Pkg.project().dependencies))
    using TestEnv; TestEnv.activate()
end

using KiteUtils
se().abs_tol=0.00000006
se().rel_tol=0.0000001

using KiteControllers, KiteModels, Plots, BayesOpt
# using Memoize

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
MAX_ITER = 80
SHOW_KITE = false
# end of user parameter section #

LAST_RES = 1e10
ITER=1
if ! @isdefined T;       const T = zeros(Int64(MAX_TIME/dt)); end
if ! @isdefined AZIMUTH; const AZIMUTH = zeros(Int64(MAX_TIME/dt)); end
if ! @isdefined P; const P = zeros(Int64(MAX_ITER+10)); end
if ! @isdefined D; const D = zeros(Int64(MAX_ITER+10)); end
if ! @isdefined RES; const RES = zeros(Int64(MAX_ITER+10)); end

function simulate(integrator)
    global  AZIMUTH
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

function calc_res(az, suppress_overshoot_factor, t1=20, t2=60)
    n1=Int(t1/dt)
    n2=Int(t2/dt)
    n = length(az)
    res = sum(abs2.(rad2deg.(az[n1:n])))/40
    overshoot = zeros(n-n1)
    for i in 1:n-n1
        if az[i] < 0
            overshoot[i] = -az[i]
        end
    end
    res += sum(abs2.(rad2deg.(overshoot)))/40 * suppress_overshoot_factor
    # if res > 500 res = 500.0 end
    res
end

# tests the parking controller and returns the sum of the square of 
# the azimuth error in degrees squared and divided by the test duration
function test_parking(suppress_overshoot_factor = 3.0)
    global LAST_RES
    global  AZIMUTH
    global  ITER
    clear!(kps)
    init_kcu(kcu, se())
    AZIMUTH .= zeros(Int64(MAX_TIME/dt))
    integrator = KiteModels.init_sim!(kps, stiffness_factor=0.04)
    simulate(integrator)
    res = calc_res(AZIMUTH, suppress_overshoot_factor)
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
    global ITER, RES
    fcs.p = x[1]
    fcs.d = x[2]
    P[ITER] = fcs.p
    D[ITER] = fcs.d
    println("x: ", x)
    res = test_parking()
    RES[ITER] = res
    ITER+=1
    res
end

function tune_1p()
    global LAST_RES
    global ITER
    ITER = 1
    LAST_RES = 1e10
    fcs.i = 0.2
    config = ConfigParameters()         # calls initialize_parameters_to_default of the C API
    config.noise = 1e-4
    config.n_iterations = MAX_ITER
    println(config.noise)
    println(config.n_inner_iterations)
    set_kernel!(config, "kMaternARD5")  # calls set_kernel of the C API
    config.sc_type = SC_MAP
    lowerbound = [10., 10.]; upperbound = [30., 50.]
    x, optimum = bayes_optimization(f, lowerbound, upperbound, config)
    fcs.p = x[1]
    fcs.d = x[2]
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

function plot_res()
    plot(1:90, P)
    plot!(1:90, D)
    plot!(1:90, RES)
end

fcs.p=13.63
fcs.i=0.0
fcs.d=27.75
println(test_parking())
show_result()
