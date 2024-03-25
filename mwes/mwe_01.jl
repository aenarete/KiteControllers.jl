using KiteViewers.GLMakie, NativeFileDialog

fig = Figure(size=(200, 200), backgroundcolor=RGBf(0.7, 0.8, 1))
btn_RUN  = Button(fig, label = " Open file... ")
lk = ReentrantLock()
filename = ""
on(btn_RUN.clicks) do c;
    println("opening file dialog...")
    Threads.@spawn begin
        global filename
        lock(lk) do
            filename      = pick_file("")
        end
    end
end
@async begin
    while true
        global filename
        lock(lk) do
            if filename != ""
                println(filename)
                filename = ""
            end
        end
        sleep(0.1)
    end
end
fig

