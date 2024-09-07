using REPL.TerminalMenus

options = ["autopilot_1p = include(\"autopilot_1p.jl\")",
           "autopilot_4p = include(\"autopilot.jl\")",
           "joystick = include(\"joystick.jl\")",
           "minipilot = include(\"minipilot.jl\")",
           "minipilot_12 = include(\"minipilot_12.jl\")",
           "parking_1p = include(\"parking_1p.jl\")",
           "parking_4p = include(\"parking_4p.jl\")",
           "parking_wind_dir = include(\"parking_wind_dir.jl\")",
           "tune_4p = include(\"tune_4p.jl\"); tune_4p()",
           "quit"]

function menu()
    active = true
    while active
        menu = RadioMenu(options, pagesize=8)
        choice = request("\nChoose function to execute or `q` to quit: ", menu)

        if choice != -1 && choice != length(options)
            eval(Meta.parse(options[choice]))
        else
            println("Left menu. Press <ctrl><d> to quit Julia!")
            active = false
        end
    end
end

menu()