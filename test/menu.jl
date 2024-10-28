using REPL.TerminalMenus

options = ["test_fpc  = include(\"test_flightpathcalculator.jl\")",
           "test_fpc1 = include(\"test_flightpathcontroller1.jl\")",
           "test_fpc2 = include(\"test_flightpathcontroller2.jl\")",
           "test_fpc3 = include(\"test_flightpathcontroller3.jl\")",
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