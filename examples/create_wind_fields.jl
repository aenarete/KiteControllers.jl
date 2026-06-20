 using Pkg
if ! ("MakieControlPlots" ∈ keys(Pkg.project().dependencies))
    Pkg.activate(@__DIR__)
end
using Timers; tic()
using KiteControllers, KiteModels

import YAML
function read_project()
    config_file = joinpath(get_data_path(), "gui.yaml")
    if ! isfile(config_file)
        cp(config_file * ".default", config_file)
    end
    dict = YAML.load_file(config_file)
    dict["gui"]["project"]
end

let
    PROJECT = read_project()
    set = load_settings(PROJECT; relax=true)
    am::AtmosphericModel = AtmosphericModel(set)
    new_windfields(am::AtmosphericModel; prn=true)
end
toc()
nothing