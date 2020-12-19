using VRPTW
using Test

@testset "VRPTW" begin
    @testset "Solomon Instance Test" begin
        solomon_dataset_name = "R102_025"
        solomon = load_solomon(solomon_dataset_name)
        vrptw = generate_solomon_vrptw_instance(solomon)

        @time routes, objective_value = solve_vrp_bnb(vrptw);
        @test isapprox(objective_value, 548.1078177906317, atol=1e-7)
    end

    include("or-tools-example.jl")
    include("espprc-example.jl")

end