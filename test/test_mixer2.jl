using KiteControllers, Plots

PERIOD_TIME = 0.05

m2 = KiteControllers.Mixer_2CH(PERIOD_TIME, 1.0)
x = ones(10)
y = 2*x
out = zeros(10)
for i in 1:length(x)
    in1=x[i]
    in2=y[i]
    out[i] = calc_output(m2, x[i], y[i])
    select_b(m2, i > 2)
    on_timer(m2)
end
plot(1:10, x, label="input_a", width=2, xtickfontsize=12, ytickfontsize=12, legendfontsize=12)
plot!(1:10, y, label="input_b", width=2)
plot!(1:10, out, label="output", width=2)
