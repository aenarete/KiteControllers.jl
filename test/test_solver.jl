# from scipy import optimize

# def fun(x):
#     return [x[0]  + 0.5 * (x[0] - x[1])**3 - 1.0,  0.5 * (x[1] - x[0])**3 + x[1]]
    
# def test_broyden2():
#     sol = optimize.broyden2(fun, [0, 0])
#     print(sol)
    
# test_broyden2()
# Output: [0.84116365 0.15883529], needs 10 iterations
# time for executing test_broyden2() loop in µs:  1276

using NLsolve
const MAX_ITER = 100

function fun!(F, x)
    F[1] = x[1]  + 0.5 * (x[1] - x[2])^3 - 1.0
    F[2] = 0.5 * (x[2] - x[1])^3 + x[2]
end

function test_nlsolve()
    sol = nlsolve(fun!, [ 0.0; 0.0], iterations=MAX_ITER)
    @assert sol.f_converged
    sol.zero
end
# execution time: 5 µs, 255 times faster than Python
