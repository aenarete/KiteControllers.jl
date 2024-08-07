__PRECOMPILE__ = true
let 
    mkdir("../output/", exist_ok=true)
    include("../examples/autopilot.jl")
end

    
@info "Precompile script has completed execution."