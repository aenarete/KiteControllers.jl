using KiteViewers.GLMakie, NativeFileDialog

fig = Figure(size=(200, 200), backgroundcolor=RGBf(0.7, 0.8, 1))
btn_RUN  = Button(fig, label = " Open file... ")
on(btn_RUN.clicks) do c;
    println("opening file dialog...")
    @async begin 
        filename = fetch(Threads.@spawn pick_file(""))
        println(filename)
    end
end
fig

