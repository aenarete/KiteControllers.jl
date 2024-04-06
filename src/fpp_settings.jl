@with_kw mutable struct FPPSettings @deftype Float64
    log_level::Int64 = 2
    min_depower::Float64 =  22 # in percent
    max_depower::Float64 =  40 
    parking_depower::Float64 = 25
    min_length::Float64 = 168.5
    max_length::Float64 = 500
    max_height::Float64 = 500
    beta_set::Float64 = 26.0
    "width of the figure of eight in degrees"
    w_fig::Float64 = 36.0
    psi_dot_max::Float64 = 3.0
    "degrees, before finishing the right and left turns"
    heading_offset_low::Float64 = 22.0
end