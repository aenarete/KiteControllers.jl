# def test2():
#     """
#     Test 2: Test the three channel mixer, using two sinus signals of different frequencies
#     and a random noise signal.
#     """
#     result = np.zeros(SAMPLES)
#     factor_b = np.zeros(SAMPLES)
#     factor_c = np.zeros(SAMPLES)
#     mix3 = Mixer_3CH()
#     for i in range(SAMPLES):
#         #if 2*i > SAMPLES and 1.5 * i < SAMPLES:
#         if almost_equal(time(i), T1):
#             mix3.selectB(True)
#         if almost_equal(time(i), T2):
#             mix3.selectC(True)
#         if almost_equal(time(i), T3):
#             mix3.selectC(False)
#         mix3.setInputA(SIN1[i])
#         mix3.setInputB(SIN2[i])
#         mix3.setInputC(NOISE[i])
#         factor_b[i] = mix3._factor_b
#         factor_c[i] = mix3._factor_c
#         mix3.onTimer()
#         result[i] = mix3.getOutput()
#     return TIME, result, factor_b, factor_c