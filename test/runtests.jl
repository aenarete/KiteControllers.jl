using KiteControllers
using Test

@testset "saturate" begin
    @test saturate( 10.0, -1,   1) == 1
    @test saturate(-10.0, -1,   1) == -1
    @test saturate( 10.0, -1.5, 1.5) == 1.5
    @test saturate( 0.5, -1.5, 1.5) == 0.5
end

# x=range(-4pi, 4pi, step=0.1)
# y=wrap2pi.(x)
# plot(x,y)

@testset "wrap2pi" begin
    @test wrap2pi(0) == 0
    @test wrap2pi(pi-0.001) == pi-0.001
    @test wrap2pi(-pi) == -pi
    @test wrap2pi(2pi) == 0
    @test wrap2pi(-2pi) == 0
end

@testset "Integrator" begin
    int1 = Integrator()
    @test int1.output == 0.0
    @test int1.last_output == 0.0
    @test int1.I == 1.0
    int2 = Integrator(2)
    @test int2.output == 0.0
    @test int2.last_output == 0.0
    @test int2.I == 2.0
    int3 = Integrator(2,2)
    @test int3.output == 2.0
    @test int3.last_output == 2.0
    @test int3.I == 2.0
    @test update(int3, 0.5, 0.05) == 2.05
    reset(int3, 1.1)
    @test int3.output == 1.1
    @test int3.last_output == 1.1
    @test int3.I == 2.0
    int3 = Integrator(2,2)
    update(int3, 0.5, 0.05) == 2.05
    on_timer(int3)
    @test int3.last_output == 2.05
end

@testset "FlightPathController" begin
    fpc = FlightPathController()    
end
