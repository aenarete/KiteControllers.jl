# activate the test environment if needed
using Pkg
if ! ("Plots" ∈ keys(Pkg.project().dependencies))
    using TestEnv; TestEnv.activate()
    Pkg.resolve()
end

@info "Loading packages ..."
using BenchmarkTools, Plots, KiteUtils, NLsolve, Parameters, StaticArrays, WinchModels, PackageCompiler, KiteViewers, 
      KiteModels, KitePodModels, StructTypes, YAML

@info "Creating sysimage ..."
push!(LOAD_PATH,joinpath(pwd(),"src"))

PackageCompiler.create_sysimage(
    [:BenchmarkTools, :Plots, :KiteUtils, :NLsolve, :Parameters, :StaticArrays, :WinchModels, :KiteViewers, 
     :KiteModels, :KitePodModels, :StructTypes, :YAML];
    sysimage_path="kps-image_tmp.so",
    precompile_execution_file=joinpath("test", "test_for_precompile.jl")
)
