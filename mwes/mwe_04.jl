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
    lscene = GLMakie.LScene(fig[1, 1], show_axis=false)
    GLMakie.text!(lscene, position =  GLMakie.Point2f(10, 340), "Uwe", fontsize = 24, space=:pixel)
    var = @sprintf("%5.0f Wh", stats.e_mech)
    # @sprintf("%5.0f  N", stats.max_force)
    # @sprintf("%5.0f  m", stats.min_height)

    display(GLMakie.Screen(), fig)
    nothing
end


