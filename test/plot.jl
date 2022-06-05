using Pkg
if ! ("Plots" ∈ keys(Pkg.project().dependencies))
    using TestEnv; TestEnv.activate()
end
using Plots; inspectdr()

x=0:0.05:100
y1=sin.(x)

p1 = plot(x,y1, size=(1200,700), legend=false)
