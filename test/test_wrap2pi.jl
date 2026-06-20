# activate the test environment if needed
using Pkg
if ! ("MakieControlPlots" ∈ keys(Pkg.project().dependencies))
     Pkg.activate(@__DIR__)
end
using Timers; tic()

using KiteControllers, MakieControlPlots

x = -4π:0.1:4π
y = wrap2pi.(x)

p=plot(x, y; ylabel="wrap2pi")
display(p)
