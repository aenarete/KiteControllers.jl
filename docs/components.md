## Generic control components
A number of components (struct Types plus functions working on these types) of this package are generic and can be used for any type of controller.

They are documented on this page.

### Types
```julia
Integrator
UnitDelay
RateLimiter
Mixer_2D
Mixer_3D
```
Usage of the Integrator
```julia
int = Integrator()  
int = Integrator(2,3) # integration constant, inital output  
reset(int)            # reset the integrator
update(int, 2)        # input value  
on_timer(int)         # must be called on each timestep
```
Usage of UnitDelay
```julia
ud = UnitDelay()
for i in 1:3
    out = calc_output(ud, i) # updates the input and calculates the output
    on_timer(ud)             # next timestep
    println(out)
end
```
Expected output: `0.0 1.0 2.0`

Usage of RateLimiter
```julia
rl = RateLimiter(0.8)
input = [0,0,1,2,3,3,2,1,0,0]
out = zeros(10)
for i in 1:10
    out[i] = calc_output(rl, input[i])
    on_timer(rl)
    println(input[i], " ", out[i])
end
```
Expected output:
```
0 0.0
0 0.0
1 0.5
2 1.0
3 1.5
3 2.0
2 2.0
1 1.5
0 1.0
0 0.5
```
Usage of Mixer_2D
```
m2 = Mixer_2D()
```

Continue with [README](../README.md)
