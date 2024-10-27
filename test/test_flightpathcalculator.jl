# activate the test environment if needed
using Pkg
if ! ("KiteModels" ∈ keys(Pkg.project().dependencies))
    using TestEnv; TestEnv.activate()
end
using Timers; tic()

using KiteControllers, KiteModels

set = deepcopy(load_settings("system.yaml"))
kcu::KCU   = KCU(set)
kps4::KPS4 = KPS4(kcu)

wcs::WCSettings = WCSettings(dt = 1/set.sample_freq)
fcs::FPCSettings = FPCSettings(dt=wcs.dt)
fpps::FPPSettings = FPPSettings()
u_d0 = 0.01 * set.depower_offset
u_d  = 0.01 * set.depower
ssc::SystemStateControl = SystemStateControl(wcs, fcs, fpps; u_d0, u_d)
dt = wcs.dt

fpca = ssc.fpp.fpca