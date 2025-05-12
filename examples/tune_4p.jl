# activate the test environment if needed
using Pkg
if VERSION.minor > 10
    error("This example is only compatible with Julia 1.10!")
end
if ! ("BayesOpt" ∈ keys(Pkg.project().dependencies))
    using TestEnv; TestEnv.activate()
    # this will not work with Julia 1.11 or newer
    Pkg.add("BayesOpt")
end

using KiteUtils
set = deepcopy(load_settings("system.yaml"))
set.abs_tol=0.0006
set.rel_tol=0.001

using KiteControllers, KiteModels, BayesOpt, ControlPlots
plt.close("all")

kcu::KCU  = KCU(set)
kps4::KPS4 = KPS4(kcu)
wcs::WCSettings   = WCSettings(true, dt = 1/set.sample_freq)
fcs::FPCSettings  = FPCSettings(true, dt=wcs.dt)
fpps::FPPSettings = FPPSettings(true)
u_d0 = 0.01 * set.depower_offset
u_d  = 0.01 * set.depower
ssc::SystemStateControl = SystemStateControl(wcs, fcs, fpps; u_d0, u_d, v_wind = set.v_wind)
dt::Float64 = wcs.dt

# the following values can be changed to match your interest
MAX_TIME::Float64 = 120
MAX_ITER          = 40
SHOW_KITE         = false
# end of user parameter section #

LAST_RES = 1e10
T::Vector{Float64} = zeros(Int64(MAX_TIME/dt))
AZIMUTH::Vector{Float64}       = zeros(Int64(MAX_TIME/dt))

function simulate(integrator)
    i=1
    sys_state = SysState(kps4)
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
            elseif time < 20.5
                steering = 0.1
            end
            set_depower_steering(kps4.kcu, depower, steering)
        end  
        v_ro = 0.0
        KiteModels.next_step!(kps4, integrator; set_speed=v_ro, dt=dt)
        sys_state = SysState(kps4)
        T[i] = dt * i
        AZIMUTH[i] = sys_state.azimuth        
        on_new_systate(ssc, sys_state)
        if i*dt >= MAX_TIME break end
        i += 1
    end
    return 1
end

function calc_res(az, suppress_overshoot_factor, t1=20)
    n1 = Int(t1/dt)
    n  = length(az)
    res = sum(abs2.(rad2deg.(az[n1:n])))/40
    overshoot = zeros(n-n1)
    for i in 1:n-n1
        if az[i] < 0
            overshoot[i] = -az[i]
        end
    end
    res += sum(abs2.(rad2deg.(overshoot)))/40 * suppress_overshoot_factor
    res
end

# tests the parking controller and returns the sum of the square of 
# the azimuth error in degrees squared and divided by the test duration
function test_parking(suppress_overshoot_factor = 3.0)
    global LAST_RES
    clear!(kps4)
    KitePodModels.init_kcu!(kcu, set)
    AZIMUTH .= zeros(Int64(MAX_TIME/dt))
    integrator = KiteModels.init_sim!(kps4, delta=0.1, stiffness_factor=1)
    simulate(integrator)
    res = calc_res(AZIMUTH, suppress_overshoot_factor)
    if res < LAST_RES
        LAST_RES = res
        println(res, " p: ", fcs.p, " i: ", fcs.i, " d: ", fcs.d)
        display(show_result())
    end
    res
end

function show_result()
    plot(T, rad2deg.(AZIMUTH); xlabel="Time [s]", ylabel="Azimuth [deg]")
end

function f(x)
    fcs.p = x[1]
    fcs.i = 0.04
    fcs.d = x[2]
    println("x: ", x)
    test_parking()
end

function tune_4p()
    global LAST_RES
    LAST_RES = 1e10
    config = ConfigParameters()         # calls initialize_parameters_to_default of the C API
    config.noise=1e-4
    config.n_iterations = MAX_ITER
    config.sc_type = SC_MAP
    set_kernel!(config, "kMaternARD5")
    println(config.noise)
    # println(config.n_inner_iterations)
    lowerbound = [0.1, 7.]; upperbound = [1.5, 16.]
    optimizer, optimum = bayes_optimization(f, lowerbound, upperbound, config)
    println("Opimal parameters: p = $(optimizer[1]),  d = $(optimizer[2])")
    println("Optimum value    : $(optimum)")
end

# fcs.p=2.255470121692552*0.7
# fcs.i=0.0
# fcs.d=38.724898029839586
fcs.p=1.2
fcs.i=0.04
fcs.d=13.25*0.95
fcs.use_chi = false

println(test_parking())
show_result()

# best query: 65.0176,1.37134, 50
# best query: 120,    3.99829, 70.3259
