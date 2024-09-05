# activate the test environment if needed
using Pkg
if ! ("ControlPlots" ∈ keys(Pkg.project().dependencies))
    using TestEnv; TestEnv.activate()
end
using Timers; tic()

using KiteControllers, ControlPlots

x = -4π:0.1:4π
y = wrap2pi.(x)

plot(x, y; ylabel="wrap2pi")
