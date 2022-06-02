using KiteControllers, Plots

x = -4π:0.1:4π
y = wrap2pi.(x)

plot(x, y, label="wrap2pi", width=2, xtickfontsize=12, ytickfontsize=12, legendfontsize=12)
