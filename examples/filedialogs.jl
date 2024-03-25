lk = ReentrantLock()
function load_plot()
    Threads.@spawn begin
        global filename
        lock(lk) do
            filename = pick_file("data"; filterlist="jld2")
        end
    end
end
@async begin
    while true
        global filename
        lock(lk) do
            if filename != ""
                println(filename)
                short_filename = replace(filename, homedir() => "~")
                KiteViewers.plot_file[] = short_filename
                viewer.menu.i_selected[] = 1
                filename = ""
            end
        end
        sleep(0.1)
    end
end
