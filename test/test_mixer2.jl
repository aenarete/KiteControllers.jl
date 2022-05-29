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