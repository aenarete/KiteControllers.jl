# activate the test environment if needed
using Pkg
if ! ("ControlPlots" ∈ keys(Pkg.project().dependencies))
    using TestEnv; TestEnv.activate()
    Pkg.resolve()
end

@info "Loading packages ..."
using BenchmarkTools, ControlPlots, KiteUtils, NLsolve, Parameters, StaticArrays, StaticArrayInterface, WinchModels, PackageCompiler, KiteViewers, 
      KiteModels, KitePodModels, StructTypes, YAML, StatsBase, ControlPlots, NativeFileDialog

@info "Creating sysimage ..."
push!(LOAD_PATH,joinpath(pwd(),"src"))

PackageCompiler.create_sysimage(
    [:BenchmarkTools, :ControlPlots, :KiteUtils, :NLsolve, :Parameters, :StaticArrays, :StaticArrayInterface, :WinchModels, :KiteViewers, 
     :KiteModels, :KitePodModels, :StructTypes, :YAML, :StatsBase, :ControlPlots, :NativeFileDialog];
    sysimage_path="kps-image_tmp.so",
    precompile_execution_file=joinpath("test", "test_for_precompile.jl")
)
