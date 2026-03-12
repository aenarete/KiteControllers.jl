# activate the test environment if needed
using Pkg
if !("NOMAD" ∈ keys(Pkg.project().dependencies))
    Pkg.activate(@__DIR__)
end

using KiteUtils
set = deepcopy(load_settings("system.yaml"))
set.abs_tol = 0.0006
set.rel_tol = 0.001

using KiteControllers, KiteModels, NOMAD, ControlPlots
using KiteControllers: calc_steering
plt.close("all")

kcu::KCU = KCU(set)
kps4::KPS4 = KPS4(kcu)
wcs::WCSettings = WCSettings(true, dt=1 / set.sample_freq)
fcs::FPCSettings = FPCSettings(true, dt=wcs.dt)
fpps::FPPSettings = FPPSettings(true)
u_d0 = 0.01 * set.depower_offset
u_d = 0.01 * set.depower
ssc::SystemStateControl = SystemStateControl(wcs, fcs, fpps; u_d0, u_d, v_wind=set.v_wind)
dt::Float64 = wcs.dt

# the following values can be changed to match your interest
MAX_TIME::Float64 = 120
MAX_ITER = 40
SHOW_KITE = false
# end of user parameter section #

LAST_RES = 1e10
T::Vector{Float64} = zeros(Int64(MAX_TIME / dt))
AZIMUTH::Vector{Float64} = zeros(Int64(MAX_TIME / dt))

function simulate(integrator)
    i = 1
    sys_state = SysState(kps4)
    on_new_systate(ssc, sys_state)
    while true
        if i > 100
            time = i * dt
            depower = max(KiteControllers.get_depower(ssc), 0.22)
            local steering
            try
                steering = calc_steering(ssc::SystemStateControl, 0)
            catch e
                println("calc_steering failed at t=$(round(time, digits=2)): $e")
                return 2  # failure code
            end
            if time >= 20 && time < 20.5
                steering = 0.1          # disturbance pulse
            end
            set_depower_steering(kps4.kcu, depower, steering)
        end
        v_ro = 0.0
        KiteModels.next_step!(kps4, integrator; set_speed=v_ro, dt=dt)
        sys_state = SysState(kps4)
        T[i] = dt * i
        AZIMUTH[i] = sys_state.azimuth
        on_new_systate(ssc, sys_state)
        if i * dt >= MAX_TIME
            break
        end
        i += 1
    end
    return 1
end

function calc_res(az, suppress_overshoot_factor, t1=5)
    n1 = Int(t1 / dt)
    n = length(az)
    res = sum(abs2.(rad2deg.(az[n1:n]))) / 40
    overshoot = zeros(n - n1)
    for i in 1:n-n1
        if az[n1+i-1] < 0
            overshoot[i] = -az[n1+i-1]
        end
    end
    res += sum(abs2.(rad2deg.(overshoot))) / 40 * suppress_overshoot_factor
    res
end

# tests the parking controller and returns the sum of the square of 
# the azimuth error in degrees squared and divided by the test duration
function test_parking(p=fcs.p, i=fcs.i, d=fcs.d, gain=fcs.gain; suppress_overshoot_factor=3.0)
    global LAST_RES, AZIMUTH
    clear!(kps4)
    KitePodModels.init_kcu!(kcu, set)
    on_parking(ssc)
    # fully reset FPC state that persists across runs:
    # - _i=0 triggers the integrator-reset logic on the first step
    # - int/int2 reset clears the I and D integrators (reset_int2 is false by default)
    # - k_u_in/k_psi_in reset gives NLsolve a clean starting point
    let fpc = ssc.fpp.fpca.fpc
        fpc._i = 0
        fpc.k_u_in = 0.0
        fpc.k_psi_in = 0.0
        reset(fpc.int, 0.0)
        reset(fpc.int2, 0.0)
    end
    # fcs === ssc.fpp.fpca.fpc.fcs (same object); set gains after on_parking to
    # ensure they are not overwritten by any reset triggered during the transition
    fcs.p = p
    fcs.i = i
    fcs.d = d
    fcs.gain = gain
    AZIMUTH .= zeros(Int64(MAX_TIME / dt))
    integrator = KiteModels.init!(kps4, delta=0.001, stiffness_factor=0.01)
    status = simulate(integrator)
    if status != 1
        return 1e6  # large penalty for failed simulation
    end
    res = calc_res(AZIMUTH, suppress_overshoot_factor)
    if res < LAST_RES
        LAST_RES = res
        println(res, " p: ", fcs.p, " i: ", fcs.i, " d: ", fcs.d)
        display(show_result(copy(T), copy(AZIMUTH)))
    end
    res
end

function show_result(t=T, az=AZIMUTH)
    plot(t, rad2deg.(az); xlabel="Time [s]", ylabel="Azimuth [deg]")
end

function f(x)
    println("x: ", x)
    test_parking(x[1], fcs.i, x[2], fcs.gain)
end

function tune_4p()
    global LAST_RES
    LAST_RES = 1e10
    lowerbound = [0.5, 10.0]
    upperbound = [12.0, 80.0]
    x0 = [fcs.p, fcs.d]
    function bb(x::Vector{Float64})
        result = f(x)
        failed = result >= 1e6
        return (!failed, !failed, [result])
    end
    p = NomadProblem(2, 1, ["OBJ"], bb;
        lower_bound=lowerbound,
        upper_bound=upperbound)
    p.options.max_bb_eval = MAX_ITER
    result = solve(p, x0)
    if !hasproperty(result, :x_sol) || isnothing(result.x_sol)
        println("NOMAD did not find a feasible solution. Status: $(result.status)")
        return
    end
    optimizer = result.x_sol
    optimum = result.bbo_sol[1]
    println("Optimal parameters: p = $(optimizer[1]),  d = $(optimizer[2])")
    println("Optimum value    : $(optimum)")
    fcs.p = optimizer[1]
    fcs.d = optimizer[2]
    println(test_parking())
    plt.close("all")
    println(" p: ", fcs.p, " i: ", fcs.i, " d: ", fcs.d)
    show_result(copy(T), copy(AZIMUTH))
end

# fcs.p=2.255470121692552*0.7
# fcs.i=0.0
# fcs.d=38.724898029839586
fcs.p = 7.79
fcs.i = -0.04
fcs.d = 33.48
fcs.use_chi = false
fcs.gain = -0.2

test_parking()
show_result(copy(T), copy(AZIMUTH))

# best query: 65.0176,1.37134, 50
# best query: 120,    3.99829, 70.3259
