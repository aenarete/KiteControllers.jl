using KiteControllers
using Test

@testset "saturate" begin
    @test saturate( 10.0, -1,   1) == 1
    @test saturate(-10.0, -1,   1) == -1
    @test saturate( 10.0, -1.5, 1.5) == 1.5
    @test saturate( 0.5, -1.5, 1.5) == 0.5
end