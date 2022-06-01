using KiteControllers, Plots

m2 = Mixer_2CH(1.0)
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
plot(1:10, x, label="input_a",xtickfontsize=12,ytickfontsize=12,legendfontsize=12)
plot!(1:10, y, label="input_b")
plot!(1:10, out, label="output")

# def test1():
#     """ Test 1: Test the two channel mixer, using two sinus signals of different frequencies. """
#     result = np.zeros(SAMPLES)
#     factor_b = np.zeros(SAMPLES)
#     mix2 = Mixer_2CH()
#     for i in range(SAMPLES):
#         #if 2*i > SAMPLES and 1.5 * i < SAMPLES:
#         if almost_equal(time(i), T1):
#             mix2.selectB(True)
#             print("selectB(true)")
#         if almost_equal(time(i), T2):
#             mix2.selectB(False)
#             print("selectB(false)")
#         mix2.setInputA(SIN1[i])
#         mix2.setInputB(SIN2[i])
#         factor_b[i] = mix2._factor_b
#         mix2.onTimer()
#         result[i] = mix2.getOutput()
#     return TIME, result, factor_b