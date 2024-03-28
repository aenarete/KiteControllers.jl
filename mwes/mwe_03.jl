using Gtk4, Printf


struct Stats
    energy::Float64
    max_force::Float64
    min_height::Float64
end

stats=Stats(333, 3900, 25)

function show_stats(stats::Stats)
    win = GtkWindow("Statistics", 400, 200)
    css = """
    label {
        font-size: 2.0em;
    }
    """
    cssProvider = GtkCssProvider(css)
    push!(Gtk4.display(win), cssProvider)

    vbox = GtkBox(:v, homogeneous=true)

    hbox = GtkBox(:h, homogeneous=true)  # :h makes a horizontal layout, :v a vertical layout
    label = GtkLabel("Power:"); val = GtkLabel(@sprintf("%5.0f Wh", stats.energy))
    push!(hbox, label, val)

    push!(vbox, hbox)
    push!(win, vbox)
    show(win)
    nothing
end


