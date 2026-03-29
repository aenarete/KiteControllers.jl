using Pkg
if ! ("ControlPlots" ∈ keys(Pkg.project().dependencies))
    Pkg.activate(@__DIR__)
end

@info "Loading packages ..."
using BenchmarkTools, ControlPlots, KiteModels, KitePodModels, KiteUtils, KiteViewers,
      NLsolve, PackageCompiler, Parameters, StaticArrays, Statistics, StructTypes,
      WinchModels, WinchControllers, YAML

@info "Creating sysimage ..."
push!(LOAD_PATH,joinpath(pwd(),"src"))

GC.gc(true)
let mem = Sys.free_memory() / 1024^2
    @info "Free memory: $(round(mem; digits=1)) MB"
    if haskey(ENV, "JULIA_IMAGE_THREADS")
        @info "JULIA_IMAGE_THREADS: $(ENV["JULIA_IMAGE_THREADS"])"
    else
        @info "JULIA_IMAGE_THREADS not defined!"
    end
end

precompile_script = joinpath("test", "precompile_all.jl")
open(precompile_script, "w") do io
    println(io, """include("test_for_precompile.jl")""")
    println(io, """include("precompile_batch_plot.jl")""")
end

PackageCompiler.create_sysimage(
    [:KiteUtils, :NLsolve, :Parameters, :StaticArrays, :WinchModels, :WinchControllers, :KiteViewers, 
     :KiteModels, :KitePodModels, :StructTypes, :YAML, :Statistics, :ControlPlots];
    sysimage_path="kps-image_tmp.so",
    include_transitive_dependencies=true,
    precompile_execution_file=precompile_script
)
