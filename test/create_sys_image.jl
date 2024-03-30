# activate the test environment if needed
using Pkg
if ! ("ControlPlots" ∈ keys(Pkg.project().dependencies))
    using TestEnv; TestEnv.activate()
    Pkg.resolve()
end

@info "Loading packages ..."
using BenchmarkTools, KiteUtils, NLsolve, Parameters, StaticArrays, StaticArrayInterface, WinchModels, PackageCompiler, KiteViewers, 
      KiteModels, KitePodModels, StructTypes, YAML, StatsBase, ControlPlots

@info "Creating sysimage ..."
push!(LOAD_PATH,joinpath(pwd(),"src"))

PackageCompiler.create_sysimage(
    [:BenchmarkTools, :KiteUtils, :NLsolve, :Parameters, :StaticArrays, :StaticArrayInterface, :WinchModels, :KiteViewers, 
     :KiteModels, :KitePodModels, :StructTypes, :YAML, :StatsBase, :ControlPlots];
    sysimage_path="kps-image_tmp.so",
    precompile_execution_file=joinpath("test", "test_for_precompile.jl")
)
