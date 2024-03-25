using KiteViewers.GLMakie, NativeFileDialog

const running    = Observable(false)
fig = Figure(size=(200, 200), backgroundcolor=RGBf(0.7, 0.8, 1))
sub_fig = fig[1,1]
fig[2, 1] = buttongrid = GridLayout(tellwidth=false)
btn_RUN  = Button(sub_fig, label = " RUN ")
buttongrid[1, 1:1] = [btn_RUN]
on(btn_RUN.clicks) do c;
    println("opening file dialog...")
    filename      = pick_file(pwd())
    println(filename)
end
fig

