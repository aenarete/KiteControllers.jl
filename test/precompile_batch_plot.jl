__PRECOMPILE__ = true
let
    include("../examples/batch_plot.jl")

    for (name, fn) in MENU_ITEMS
        println("Precompiling $name ...")
        try
            fn()
        catch e
            println("  Skipped $name: $e")
        end
    end

    # Also exercise plot_timing / plot_timing2 (not in MENU_ITEMS)
    for (name, fn) in [("plot_timing", plot_timing), ("plot_timing2", plot_timing2)]
        println("Precompiling $name ...")
        try
            fn()
        catch e
            println("  Skipped $name: $e")
        end
    end
end

@info "Precompile script precompile_batch_plot.jl has completed execution."
