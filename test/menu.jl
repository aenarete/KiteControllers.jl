using REPL.TerminalMenus

options = ["test_fpc  = include(\"test_flightpathcalculator.jl\")",
           "test_fpc1 = include(\"test_flightpathcontroller1.jl\")",
           "test_fpc2 = include(\"test_flightpathcontroller2.jl\")",
           "test_fpc3 = include(\"test_flightpathcontroller3.jl\")",
           "test_forcespeed1 = include(\"test_forcespeedcontroller1.jl\")",
           "test_forcespeed2 = include(\"test_forcespeedcontroller2.jl\")",
           "test_fpc4 = include(\"test_fpc_low_right.jl\")",
           "test_lower_force1 = include(\"test_lower_force1.jl\")",
           "test_mixer2 = include(\"test_mixer2.jl\")",
           "test_mixer3 = include(\"test_mixer3.jl\")",
           "test_solver = include(\"test_solver.jl\")",
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