using Pkg
if ! ("KiteViewers" ∈ keys(Pkg.project().dependencies))
    using TestEnv; TestEnv.activate()
end
using Printf
import KiteViewers.GLMakie

struct Stats
    e_mech::Float64
    max_force::Float64
    min_height::Float64
end

stats = Stats(333, 3900, 25)

function show_stats(stats::Stats)
    fig = GLMakie.Figure(size = (400, 400))
    GLMakie.text!(fig.scene, 20, 0, text="hello", fontsize = 30, space=:pixel)
    display(GLMakie.Screen(), fig)
    nothing
end
