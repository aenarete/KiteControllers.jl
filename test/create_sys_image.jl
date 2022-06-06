# activate the test environment if needed
using Pkg
if ! ("Plots" ∈ keys(Pkg.project().dependencies))
    using TestEnv; TestEnv.activate()
end

@info "Loading packages ..."
using BenchmarkTools, Plots, KiteUtils, NLsolve, Parameters, StaticArrays, WinchModels, PackageCompiler

@info "Creating sysimage ..."
push!(LOAD_PATH,joinpath(pwd(),"src"))

PackageCompiler.create_sysimage(
    [:BenchmarkTools, :Plots, :KiteUtils, :NLsolve, :Parameters, :StaticArrays, :WinchModels];
    sysimage_path="kps-image_tmp.so",
    precompile_execution_file=joinpath("test", "test_for_precompile.jl")
)
