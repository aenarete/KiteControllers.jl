@with_kw mutable struct FPPSettings @deftype Float64
    log_level::Int64 = 2
    min_depower::Float64 =  22 # in percent
    max_depower::Float64 =  40 
    parking_depower::Float64 = 25
    min_length::Float64 = 168.5
    max_length::Float64 = 500
    max_height::Float64 = 500
    beta_set::Float64 = 26.0
end