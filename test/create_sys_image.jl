using Pkg

@info "Loading packages ..."
using BenchmarkTools, KiteUtils, NLsolve, Parameters, StaticArrays, WinchModels, PackageCompiler, KiteViewers, 
      KiteModels, KitePodModels, StructTypes, YAML, StatsBase, ControlPlots

@info "Creating sysimage ..."
push!(LOAD_PATH,joinpath(pwd(),"src"))

PackageCompiler.create_sysimage(
    [:BenchmarkTools, :KiteUtils, :NLsolve, :Parameters, :StaticArrays, :WinchModels, :KiteViewers, 
     :KiteModels, :KitePodModels, :StructTypes, :YAML, :StatsBase, :ControlPlots];
    sysimage_path="kps-image_tmp.so",
    include_transitive_dependencies=true,
    precompile_execution_file=joinpath("test", "test_for_precompile.jl")
)
