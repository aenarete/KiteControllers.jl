# provides a terminal menu with the options:
# - "select_project": calls select_project() to select a project from the data directory
# - "clear_corrections": clears all corrections from the current project
# - "train()": runs the training loop from learn_corrections.jl
# - "quit": exits the menu

using REPL.TerminalMenus

include("select_project.jl")
include("learn_corrections.jl")

function clear_corrections()
    project = read_project()
    KiteUtils.PROJECT = project
    KiteControllers.save_corr([0.0])
    println("Correction vector cleared (set to [0.0]) in $(KiteControllers.fpp_settings()).")
end

function open_documentation()
    println("\nOpening documentation in browser...")
    doc_url = "https://opensourceawe.github.io/KiteControllers.jl/dev/"
    
    try
        if Sys.islinux()
            run(pipeline(`xdg-open $doc_url`, stdout=devnull, stderr=devnull))
        elseif Sys.iswindows()
            run(pipeline(`cmd /c start $doc_url`, stdout=devnull, stderr=devnull))
        elseif Sys.isapple()
            run(pipeline(`open $doc_url`, stdout=devnull, stderr=devnull))
        else
            println("Cannot automatically open browser on this system.")
            println("Please manually open: $doc_url")
        end
        println("Documentation URL: $doc_url")
    catch _
        println("Could not open browser automatically.")
        println("Please manually open: $doc_url")
    end
end


options = ["select_project()",
           "clear_corrections()",
           "train()",
           "plot()",
           "residual(full_sim=true)",
           "plot(full_sim=true)",
           "include(\"autopilot.jl\")",
           "open_documentation()",
           "quit"]

function learning_menu()
    active = true
    while active
        menu = RadioMenu(options, pagesize=9)
        choice = request("\nProject: $(read_project())  — Choose function to execute or `q` to quit: ", menu)

        if choice != -1 && choice != length(options)
            eval(Meta.parse(options[choice]))
        else
            println("Left menu. Press <ctrl><d> to quit Julia!")
            active = false
        end
    end
end

learning_menu()