using VRPTW
using Test


test_start_time = time()

@testset "VRPTW" begin
    @testset "Solomon Instance Test" begin
        solomon_dataset = Dict(
            "C101_025" => 191.30,
            "C102_025" => 190.30,
            "C201_025" => 214.70,
            "RC105_025" => 411.30,
            "R101_025" => 617.10,
            "R102_025" => 547.10,
        )

        for (name, val) in solomon_dataset
            @info("Solving the Solomon Instance $name")
            solomon = load_solomon(name)
            vrptw = generate_solomon_vrptw_instance(solomon)

            @testset "$name" begin
                println("-- Solomon Instance $name --")
                @time routes, objective_value = solve_vrp_bnb(vrptw);
                @test isapprox(objective_value, val, atol=1e-7)
            end
        end
    end

    include("or-tools-example.jl")
    include("espprc-example.jl")

end

total_test_time = time() - test_start_time
@show total_test_time