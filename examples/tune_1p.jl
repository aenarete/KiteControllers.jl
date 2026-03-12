# activate the test environment if needed
using Pkg
if !("NOMAD" ∈ keys(Pkg.project().dependencies))
    Pkg.activate(@__DIR__)
end

using KiteUtils

set = deepcopy(load_settings("system.yaml"))
set.abs_tol = 0.00006
set.rel_tol = 0.0001

using KiteControllers, KiteModels, NOMAD, ControlPlots
using KiteControllers: calc_steering

kcu::KCU = KCU(set)
kps::KPS3 = KPS3(kcu)
wcs::WCSettings = WCSettings(true, dt=1 / set.sample_freq)
fcs::FPCSettings = FPCSettings(true, dt=wcs.dt)
fpps::FPPSettings = FPPSettings(true)
u_d0 = 0.01 * set.depower_offset
u_d = 0.01 * set.depower
ssc::SystemStateControl = SystemStateControl(wcs, fcs, fpps; u_d0, u_d, v_wind=set.v_wind)
dt::Float64 = wcs.dt

# the following values can be changed to match your interest
MAX_TIME::Float64 = 60
MAX_ITER = 200
SHOW_KITE = false
# end of user parameter section #

LAST_RES = Ref(1e10)
T::Vector{Float64} = zeros(Int64(MAX_TIME / dt))
AZIMUTH::Vector{Float64} = zeros(Int64(MAX_TIME / dt))

function simulate(integrator)
    i = 1
    sys_state = SysState(kps)
    on_new_systate(ssc, sys_state)
    while true
        if i > 100
            depower = max(KiteControllers.get_depower(ssc), 0.22)
            local steering
            try
                steering = calc_steering(ssc, 0)
            catch e
                println("calc_steering failed at t=$(round(i*dt, digits=2)): $e")
                return 2
            end
            time = i * dt
            if time >= 20 && time < 20.5
                steering = 0.1          # disturbance pulse
            end
            set_depower_steering(kps.kcu, depower, steering)
        end
        v_ro = 0.0
        KiteModels.next_step!(kps, integrator; set_speed=v_ro, dt=dt)
        sys_state = SysState(kps)
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

function calc_res(az, suppress_overshoot=10, suppress_tail=10, t1=20, t2=40)
    n1 = Int(t1 / dt)
    n2 = Int(t2 / dt)
    n = length(az)
    res = sum(abs2.(rad2deg.(az[n1:n]))) / 40
    overshoot = zeros(n - n1)
    for i in 1:(n-n1)
        if az[i+n1] < 0
            overshoot[i] = -az[i+n1]
        end
    end
    res += sum(abs.(rad2deg.(overshoot))) / 40 * suppress_overshoot
    tail = zeros(n - n2)
    for i in 1:(n-n2)
        tail[i] = abs(az[i+n2])
    end
    tail = sum(rad2deg.(tail)) / 20 * suppress_tail
    res += tail
    # println("res: $(res), tail: $tail")
    res
end

# tests the parking controller and returns the sum of the square of 
# the azimuth error in degrees squared and divided by the test duration
function test_parking(LAST_RES, p=fcs.p, i=fcs.i, d=fcs.d; suppress_overshoot=10, suppress_tail=10)
    clear!(kps)
    KitePodModels.init_kcu!(kcu, set)
    let fpc = ssc.fpp.fpca.fpc
        fpc._i = 0
        fpc.k_u_in = 0.0
        fpc.k_psi_in = 0.0
        WinchControllers.reset(fpc.int, 0.0)
        WinchControllers.reset(fpc.int2, 0.0)
    end
    fcs.p = p
    fcs.i = i
    fcs.d = d
    AZIMUTH .= zeros(Int64(MAX_TIME / dt))
    integrator = KiteModels.init!(kps, stiffness_factor=0.04)
    status = simulate(integrator)
    if status != 1
        return 1e6
    end
    res = calc_res(AZIMUTH, suppress_overshoot, suppress_tail)
    if res < LAST_RES[]
        LAST_RES[] = res
        println(res, " p: ", fcs.p, " d: ", fcs.d)
        display(show_result(copy(T), copy(AZIMUTH)))
    end
    res
end

function show_result(t=T, az=AZIMUTH)
    plot(t, rad2deg.(az); xlabel="Time [s]", ylabel="Azimuth [deg]")
end

function f(x)
    p = x[1]
    d = max(x[2] * x[1], 0.0)
    println("x: ", x)
    test_parking(LAST_RES, p, fcs.i, d)
end

function tune_1p!(LAST_RES)
    LAST_RES[] = 1e10
    lowerbound = [8., 0.]
    upperbound = [30., 1.8]
    x0 = [fcs.p, fcs.d / max(fcs.p, 1e-6)]
    function bb(x::Vector{Float64})
        res = f(x)
        failed = res >= 1e6
        return (!failed, !failed, [res])
    end
    p = NomadProblem(2, 1, ["OBJ"], bb;
        lower_bound=lowerbound,
        upper_bound=upperbound)
    p.options.max_bb_eval = MAX_ITER
    p.options.display_degree = 0
    result = solve(p, x0)
    if !hasproperty(result, :x_sol) || isnothing(result.x_sol)
        println("NOMAD did not find a feasible solution. Status: $(result.status)")
        return
    end
    optimizer = result.x_sol
    optimum = result.bbo_sol[1]
    println("Optimal parameters: p = $(optimizer[1]),  d/p = $(optimizer[2])")
    println("Optimum value     : $(optimum)")
    fcs.p = optimizer[1]
    fcs.d = optimizer[2] * optimizer[1]
    test_parking(LAST_RES)
end

fcs.p = 12.0
fcs.i = 0.5
fcs.d = fcs.p * 0.98
on_parking(ssc)
test_parking(LAST_RES)
show_result(copy(T), copy(AZIMUTH))
println()
@info "Use 'tune_1p!(LAST_RES)' to start the tuning process (this may take a while depending on MAX_ITER)."
