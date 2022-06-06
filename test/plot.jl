using Pkg
if ! ("Plots" ∈ keys(Pkg.project().dependencies))
    using TestEnv; TestEnv.activate()
end
using Plots; inspectdr()
InspectDR.defaults.xaxiscontrol_visible = false

x=0:0.05:100
y1=sin.(x)

p1 = plot(x,y1, width=2, xtickfontsize=12, ytickfontsize=12, legendfontsize=12, legend=false)
pIDR = display(p1)           # Display with InspectDR and keep plot object
resize!(pIDR.wnd, 1200, 700) # Resize GTK window directly
