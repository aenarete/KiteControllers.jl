using NLsolve
MAX_ITER = 100

function fun!(F, x)
    F[1] = x[1]  + 0.5 * (x[1] - x[2])^3 - 1.0
    F[2] = 0.5 * (x[2] - x[1])^3 + x[2]
end

function test_nlsolve()
    sol = nlsolve(fun!, [ 0.0; 0.0], iterations=MAX_ITER)
    @assert sol.f_converged
    sol.zero
end
test_nlsolve()
@time test_nlsolve()
# execution time: 25 µs, 41 times faster than Python
