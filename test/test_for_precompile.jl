MAX_TIME::Float64 = 5
let
    include("test_speedcontroller1.jl")
    include("../examples/autopilot.jl")
end

    
@info "Precompile script has completed execution."