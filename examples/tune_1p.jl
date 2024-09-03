# activate the test environment if needed
using Pkg
if ! ("ControlPlots" ∈ keys(Pkg.project().dependencies))
    using TestEnv; TestEnv.activate()
end

using KiteUtils

set = deepcopy(load_settings("system.yaml"))
set.abs_tol=0.0000006
set.rel_tol=0.000001

using KiteControllers, KiteModels, BayesOpt, ControlPlots

kcu::KCU  = KCU(set)
kps::KPS3 = KPS3(kcu)
wcs::WCSettings   = WCSettings();  wcs.dt = 1/set.sample_freq
fcs::FPCSettings  = FPCSettings(wcs.dt)
fpps::FPPSettings = FPPSettings()
u_d0 = 0.01 * set.depower_offset
u_d  = 0.01 * set.depower
ssc::SystemStateControl = SystemStateControl(wcs, fcs, fpps; u_d0, u_d)
dt::Float64 = wcs.dt

# the following values can be changed to match your interest
MAX_TIME::Float64 = 60
TIME_LAPSE_RATIO  =  1
MAX_ITER          = 60
SHOW_KITE         = false
# end of user parameter section #

LAST_RES = 1e10
ITER = 1
if ! @isdefined T;       const T = zeros(Int64(MAX_TIME/dt)); end
if ! @isdefined AZIMUTH; const AZIMUTH = zeros(Int64(MAX_TIME/dt)); end
if ! @isdefined P; const P = zeros(Int64(MAX_ITER+11)); end
if ! @isdefined D; const D = zeros(Int64(MAX_ITER+11)); end
if ! @isdefined RES; const RES = zeros(Int64(MAX_ITER+11)); end

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
        KiteModels.next_step!(kps, integrator; set_speed=v_ro, dt=dt)
        sys_state = SysState(kps)
        T[i] = dt * i
        AZIMUTH[i] = sys_state.azimuth        
        on_new_systate(ssc, sys_state)
        if i*dt >= MAX_TIME break end
        i += 1
    end
    return 1
end

function calc_res(az, suppress_overshoot=10, suppress_tail=10, t1=20, t2=40)
    n1=Int(t1/dt)
    n2=Int(t2/dt)
    n = length(az)
    res = sum(abs2.(rad2deg.(az[n1:n])))/40
    overshoot = zeros(n-n1)
    for i in 1:(n-n1)
        if az[i+n1] < 0
            overshoot[i] = -az[i+n1]
        end
    end
    res += sum(abs.(rad2deg.(overshoot)))/40 * suppress_overshoot
    tail = zeros(n-n2)
    for i in 1:(n-n2)
        tail[i] = abs(az[i+n2])
    end
    tail = sum(rad2deg.(tail))/20 * suppress_tail
    res += tail
    println("res: $(res), tail: $tail")
    res = 1.0 - 100 / res
    res
end

# tests the parking controller and returns the sum of the square of 
# the azimuth error in degrees squared and divided by the test duration
function test_parking()
    global LAST_RES
    global  AZIMUTH
    global  ITER
    clear!(kps)
    KitePodModels.init_kcu!(kcu, se())
    AZIMUTH .= zeros(Int64(MAX_TIME/dt))
    integrator = KiteModels.init_sim!(kps, stiffness_factor=0.04)
    simulate(integrator)
    res = calc_res(AZIMUTH)
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
    fcs.i = 0.5
    fcs.d = x[2]*x[1]
    if fcs.d < 0; fcs.d = 0; end
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
    fcs.i = 0.5
    config = ConfigParameters()         # calls initialize_parameters_to_default of the C API
    config.noise = 1e-4
    config.n_iterations = MAX_ITER
    println(config.noise)
    println(config.n_inner_iterations)
    set_kernel!(config, "kMaternARD5")  # calls set_kernel of the C API
    config.sc_type = SC_MAP
    lowerbound = [10., 0.]; upperbound = [30., 1.8]
    x, optimum = bayes_optimization(f, lowerbound, upperbound, config)
    fcs.p = x[1]
    fcs.d = x[2]*x[1]
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
    plot(1:71, [P, D, 10*RES])
end

fcs.p=17.33 # 14.36 # 15.61 # 14.43 # 13.65 # 14.08 # 14.72 # 15.41 # 14.74 # 14.35 # 13.68 # 13.87 # 14.99 # 13.63
fcs.i=0.5
fcs.d=23.88 # 18.81 # 21.03 # 22.45 # 22.74 # 21.28 # 16.67 # 20.89 # 25.97 # 19.61 # 25.94 # 50 # 23.09 # 27.75
println(test_parking())
show_result()
