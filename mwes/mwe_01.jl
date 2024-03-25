using KiteViewers.GLMakie, NativeFileDialog

const running    = Observable(false)
fig = Figure(size=(200, 200), backgroundcolor=RGBf(0.7, 0.8, 1))
btn_RUN  = Button(fig, label = " Open file... ")
on(btn_RUN.clicks) do c;
    println("opening file dialog...")
    # @threadcall pick_file()
    filename      = pick_file("")
    println(filename)
end
fig

