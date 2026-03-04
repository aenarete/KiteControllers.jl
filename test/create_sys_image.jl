using Pkg
if ! ("ControlPlots" ∈ keys(Pkg.project().dependencies))
    Pkg.activate(@__DIR__)
end

@info "Loading packages ..."
using BenchmarkTools, ControlPlots, KiteModels, KitePodModels, KiteUtils, KiteViewers,
      NLsolve, PackageCompiler, Parameters, StaticArrays, Statistics, StructTypes,
      WinchModels, YAML

@info "Creating sysimage ..."
push!(LOAD_PATH,joinpath(pwd(),"src"))

PackageCompiler.create_sysimage(
    [:BenchmarkTools, :KiteUtils, :NLsolve, :Parameters, :StaticArrays, :WinchModels, :KiteViewers, 
     :KiteModels, :KitePodModels, :StructTypes, :YAML, :Statistics, :ControlPlots];
    sysimage_path="kps-image_tmp.so",
    include_transitive_dependencies=true,
    precompile_execution_file=joinpath("test", "test_for_precompile.jl")
)
