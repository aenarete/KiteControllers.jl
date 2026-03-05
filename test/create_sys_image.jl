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

precompile_script = joinpath("test", "precompile_all.jl")
open(precompile_script, "w") do io
    println(io, """include("test_for_precompile.jl")""")
    println(io, """include("precompile_batch_plot.jl")""")
end

PackageCompiler.create_sysimage(
    [:BenchmarkTools, :KiteUtils, :NLsolve, :Parameters, :StaticArrays, :WinchModels, :KiteViewers, 
     :KiteModels, :KitePodModels, :StructTypes, :YAML, :Statistics, :ControlPlots];
    sysimage_path="kps-image_tmp.so",
    include_transitive_dependencies=true,
    precompile_execution_file=precompile_script
)
