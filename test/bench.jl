using BenchmarkTools

include("test_solver.jl")

@benchmark test_nlsolve()