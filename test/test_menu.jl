using REPL.TerminalMenus

function _test_scripts(test_dir::AbstractString)
    scripts = filter(readdir(test_dir; join=false)) do name
        startswith(name, "test_") && endswith(name, ".jl") && isfile(joinpath(test_dir, name))
    end
    # Avoid offering this menu script itself to prevent accidental recursion.
    filter!(name -> name != basename(@__FILE__), scripts)
    sort!(scripts)
    return scripts
end

function test_menu()
    test_dir = @__DIR__

    active = true
    while active
        scripts = _test_scripts(test_dir)
        if isempty(scripts)
            println("No test scripts matching test_*.jl found in $(test_dir).")
            break
        end

        options = vcat(["include(\"$(name)\")" for name in scripts], ["quit"])
        menu = RadioMenu(options, pagesize=min(12, length(options)))
        choice = request("\nChoose test script to include or `q` to quit: ", menu)

        if choice != -1 && choice != length(options)
            include(joinpath(test_dir, scripts[choice]))
        else
            println("Left menu. Press <ctrl><d> to quit Julia!")
            active = false
        end
    end
end

test_menu()
