let
    include("test_speedcontroller1.jl")
    MAX_TIME = 5
    include("../examples/autopilot.jl")
end

    
@info "Precompile script has completed execution."