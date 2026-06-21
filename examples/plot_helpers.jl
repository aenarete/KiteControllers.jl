# Shared helper accessors used by both plots.jl and batch_plot.jl

function l_tether(sl)
    hcat(sl.l_tether...)[1,:]
end

function force(sl)
    hcat(sl.winch_force...)[1,:]
end

function v_reelout(sl)
    hcat(sl.v_reelout...)[1,:]
end