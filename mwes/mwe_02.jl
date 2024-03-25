using Gtk
  
if !isinteractive()
    @async Gtk.gtk_main()
end

t = @async begin
  state = ask_dialog("Find Steady-state solution and start rom steady-state?", "No", "Yes")
end

# if !isinteractive()
#     wait(t)
# end