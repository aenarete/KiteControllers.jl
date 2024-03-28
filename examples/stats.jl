struct Stats
    e_mech::Float64
    max_force::Float64
    min_height::Float64
end

function show_stats(stats::Stats)
    fig = GLMakie.Figure(size = (400, 400))
    lscene = GLMakie.LScene(fig[1, 1], show_axis=false)
    if Sys.islinux()
        lin_font="/usr/share/fonts/truetype/ttf-bitstream-vera/VeraMono.ttf"
        if isfile(lin_font)
            font=lin_font
        else
            font="/usr/share/fonts/truetype/freefont/FreeMono.ttf"
        end
    else
        font="Courier New"
    end
    function print(lbl::String, value::String; line, font=font)
        GLMakie.text!(lscene, position =  GLMakie.Point2f( 10, 360-line*32), lbl; fontsize = 24, space=:pixel)
        GLMakie.text!(lscene, position =  GLMakie.Point2f(250, 360-line*32), value; fontsize = 24, font, space=:pixel)
        line +=1    
    end
    line = print("energy:    ", @sprintf("%5.0f Wh", stats.e_mech); line = 1)
    line = print("max force: ", @sprintf("%5.0f  N", stats.max_force); line)
    line = print("min height:", @sprintf("%5.0f  m", stats.min_height); line)

    display(GLMakie.Screen(), fig)
    nothing
end
