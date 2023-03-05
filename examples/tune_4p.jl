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
if ! @isdefined kps;   const kps = KPS4(kcu); end
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
function test_parking()
    clear!(kps)
    init_kcu(kcu, se())
    AZIMUTH .= zeros(Int64(MAX_TIME/dt))
    integrator = KiteModels.init_sim!(kps, stiffness_factor=0.04)
    simulate(integrator)
    res=sum(abs2.(rad2deg.(AZIMUTH)))/40
    if res < 200
        println(res, " p: ", fcs.p, " i: ", fcs.i, " d: ", fcs.d)
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

function tune_4p()
    config = ConfigParameters()         # calls initialize_parameters_to_default of the C API
    config.noise=1e-2
    # config.n_inner_iterations=UInt(95)
    println(config.noise)
    # println(config.n_inner_iterations)
    lowerbound = [10., 0., 0.]; upperbound = [100., 4.0, 40.]
    optimizer, optimum = bayes_optimization(f, lowerbound, upperbound, config)
    println("Opimal parameters: p = $(optimizer[1]), i = $(optimizer[2]), d = $(optimizer[3])")
    println("Optimum value    : $(optimum)")
end

fcs.p=100   *0.6
fcs.i=0.0
fcs.d=35.81 *0.6
println(test_parking())
show_result()
