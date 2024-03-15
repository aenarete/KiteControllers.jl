using Pkg
if ! ("Plots" ∈ keys(Pkg.project().dependencies))
    using TestEnv; TestEnv.activate()
end
import PyPlot; 

function plot1(X, Y; label="", width=2, xtickfontsize=12, ytickfontsize=12, legendfontsize=12, fig="")
    if fig != ""
        PyPlot.figure(fig)
    end
    p = PyPlot.plot(X, Y; label)
    PyPlot.grid(true)
    p
end

function plot2(X, Y1, Y2; labels=["", ""], fig="", title="")
    fig_ = PyPlot.figure(fig, figsize=(8, 6))
    ax1 = PyPlot.subplot(211) 
    PyPlot.suptitle(title, fontsize=14) # Super title
    PyPlot.plot(X, Y1, label=labels[1]); 
    ylabel(labels[1], fontsize=14);          
    PyPlot.grid(true)
    setp(ax1.get_xticklabels(), visible=false)
    ax2 = PyPlot.subplot(212, sharex = ax1)
    PyPlot.plot(X, Y2, label=labels[2])
    PyPlot.grid(true)
    ylabel(labels[2], fontsize=14);    grid(true)
    xlabel("time [s]", fontsize=14)
    xlim(0, X[end])
    tight_layout()
end