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