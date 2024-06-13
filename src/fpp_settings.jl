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
    # minimal turn radius in degrees
    r_min = 3.0
    # maximal turn radius in degrees
    r_max = 4.5
    "degrees, before finishing the right and left turns"
    heading_offset_low::Float64 = 22.0
    "dito, for the turn around the intermediate point"
    heading_offset_int::Float64 =  32.0
    "dito, for elevation angles > 47.5 degrees"
    heading_offset_high::Float64 = 54.0
    "degrees, before finishing the up-turn"
    heading_offset_up::Float64 = 60.0
    heading_upper_turn::Float64 = 360.0-25.0
    k_factor::Float64 = 1.0
    timeout::Float64 = 145
    corr_vec::Vector{Float64} = [24.02, 36.08, 32.70, 25.40, 19.85, 16.13, 13.55, 11.42, 9.32, 7.38, 5.58, 3.84]
end

StructTypes.StructType(::Type{FPPSettings}) = StructTypes.Mutable()

function update(fpps::FPPSettings)
    config_file = joinpath(get_data_path(), fpp_settings())
    if Sys.iswindows()
        config_file = replace(config_file, "/" => "\\")
    end
    if ! isfile(config_file)
        println("Warning: $config_file not found, using default settings.")
        return
    end
    dict = YAML.load_file(config_file)
    sec_dict = Dict(Symbol(k) => v for (k, v) in dict["fpp_settings"])
    StructTypes.constructfrom!(fpps, sec_dict)
end