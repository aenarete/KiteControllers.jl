using Pkg
if ! ("Gtk4" ∈ keys(Pkg.project().dependencies))
    using TestEnv; TestEnv.activate()
end
using Gtk4, Printf

struct Stats
    energy::Float64
    max_force::Float64
    min_height::Float64
end

stats = Stats(333, 3900, 25)

function show_stats(stats::Stats)
    win = GtkWindow("Statistics", 400, 200)
    css = """
    label {
        font-size: 2.0em;
        font-family: monospace;
    }
    """
    cssProvider = GtkCssProvider(css)
    push!(Gtk4.display(win), cssProvider)

    vbox = GtkBox(:v, homogeneous=true)

    function add_hbox(vbox, label, value)
        hbox = GtkBox(:h, homogeneous=true)  # :h makes a horizontal layout, :v a vertical layout
        lbl = GtkLabel(label)
        push!(hbox, lbl, value)
        push!(vbox, hbox)
    end

    add_hbox(vbox, "  energy:     ", GtkLabel(@sprintf("%5.0f Wh", stats.energy)))
    add_hbox(vbox, "  max force:  ", GtkLabel(@sprintf("%5.0f  N", stats.max_force)))
    add_hbox(vbox, "  min height: ", GtkLabel(@sprintf("%5.0f  m", stats.min_height)))


    push!(win, vbox)
    show(win)
    nothing
end


