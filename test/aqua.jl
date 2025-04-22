using Aqua
@testset "Aqua.jl" begin
    Aqua.test_all(
      KiteControllers;
      stale_deps=(ignore=[:PyCall, :REPL],), # CodecXz is used during precompilation only
      deps_compat=(ignore=[:PyCall],),                 # PyCall is needed for CI to recompile Python
      #piracies=false                                   # the norm function is doing piracy for performance reasons
    )
end
