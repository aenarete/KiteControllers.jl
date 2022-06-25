# activate the test environment if needed
using Pkg
if ! ("Plots" ∈ keys(Pkg.project().dependencies))
    using TestEnv; TestEnv.activate()
end
using Timers; tic()

using KiteControllers, KiteViewers, KiteModels

# change this to KPS3 or KPS4
const Model = KPS4

if ! @isdefined kcu;    const kcu = KCU(se());   end
if ! @isdefined kps4;   const kps4 = Model(kcu); end

wcs = WCSettings(); wcs.dt = 1/se().sample_freq
const fcs = FPCSettings(); fcs.dt = wcs.dt
const fpps = FPPSettings()
const ssc = SystemStateControl(wcs, fcs, fpps)
dt = wcs.dt

fpca = ssc.fpp.fpca