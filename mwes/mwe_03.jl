using Gtk4, Printf


struct Stats
    energy::Float64
end

stats=Stats(333)

function show_stats(stats::Stats)
    win = GtkWindow("My First Gtk4.jl Program", 400, 200)
    css = """
    button {
        font-size: 3.0em;
    }
    """
    cssProvider = GtkCssProvider(css)
    push!(Gtk4.display(win), cssProvider)

    hbox = GtkBox(:h)  # :h makes a horizontal layout, :v a vertical layout
    push!(win, hbox)

    label = GtkButton("  Power:")
    value = GtkButton("  22.4")
    push!(hbox, label)
    push!(hbox, value)
    hbox.spacing = 10
    hbox.homogeneous = true
    show(win)
    nothing
end

