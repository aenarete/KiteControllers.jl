# activate the test environment if needed
using Pkg
if ! ("KiteModels" ∈ keys(Pkg.project().dependencies))
    using TestEnv; TestEnv.activate()
end
using Timers; tic()

using KiteControllers, KiteModels

kcu::KCU = KCU(se())
kps4::KPS4 = KPS4(kcu)

wcs = WCSettings(); wcs.dt = 1/se().sample_freq
const fcs = FPCSettings(); fcs.dt = wcs.dt
const fpps = FPPSettings()
const ssc = SystemStateControl(wcs, fcs, fpps)
dt = wcs.dt

fpca = ssc.fpp.fpca