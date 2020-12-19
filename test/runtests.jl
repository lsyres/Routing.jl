using VRPTW
using Test

include("solve_solomon_vrptw.jl")

@testset "Solomon Instance Test" begin
    # Write your tests here.
    solomon = generate_solomon_vrptw_instance("R102_025")
    @time sol_routes, sol_obj = solve_vrp_bnb(solomon.vrptw, tw_reduce=false);
    @test isapprox(sol_obj, 548.1078177906317, atol=1e-7)

end

include("or-tools-example.jl")
