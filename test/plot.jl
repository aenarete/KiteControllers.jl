using Plots; pyplot()

x=0:0.05:100
y1=sin.(x)
y2=cos.(x)


p1 = plot(x,y1, size=(640,480), legend=false)
p2 = plot(x,y2, size=(640,480), reuse=false, legend=false)
display(p1); p2
