# Solving ESPPRC 
# The example given by Google OR-Tools https://developers.google.com/optimization/routing/vrp
# Modified

using VRPTW
# include("../src/VRPTWinclude.jl")
using Test 

# For testing purpose
using Random
# Random.seed!(23423) # bug instance 
Random.seed!(1)

# For Debugging
include("debugging.jl")

solomon_dataset_name = "R102_100"
solomon = load_solomon(solomon_dataset_name)
n_nodes = solomon.nodes |> length
dual_var = (rand(n_nodes) * 10)
dual_var = [dual_var; 0.0; 0.0]
pg = solomon_to_espprc(solomon, dual_var)

##################################################

@info("ESPPRC $(solomon_dataset_name) testing...")
@time sol = solveESPPRC(pg, method="pulse")
@time lab1 = solveESPPRC(pg, method="monodirectional")
@time lab1d = solveESPPRC(pg, method="monodirectional", DSSR=true)
@time lab2 = solveESPPRC(pg, method="bidirectional")
@time lab2d = solveESPPRC(pg, method="bidirectional", DSSR=true)

@show sol.cost, lab1.cost, lab1d.cost, lab2.cost, lab2d.cost

@show sol.cost, sol.load, sol.time
@show lab1.cost, lab1.load, lab1.time
@show lab2.cost, lab2.load, lab2.time
@show sol.path
@show lab1.path
@show lab2.path

@testset "ESPPRC $(solomon_dataset_name) Test" begin
    @test isapprox(sol.cost, lab1.cost, atol=1e-7)
end

println("done")

# show_details(sol.path, pg)
# show_details(lab1.path, pg)
# show_details(lab2.path, pg)

############################################################
max_neg = 20
@info("ESPPRC $(solomon_dataset_name) testing with max_neg_cost_routes=$(max_neg)...")

@time sol, neg_sols = solveESPPRC(pg, method="pulse", max_neg_cost_routes=max_neg)
@time lab1, neg_labs1 = solveESPPRC(pg, method="monodirectional", max_neg_cost_routes=max_neg)
@time lab1d, neg_labs1d = solveESPPRC(pg, method="monodirectional", max_neg_cost_routes=max_neg, DSSR=true)

# @time lab2, neg_labs2 = solveESPPRC(pg, method="bidirectional", max_neg_cost_routes=max_neg)

@testset "ESPPRC $(solomon_dataset_name) Test with max_neg_cost_routes" begin
    @test length(neg_sols) <= max_neg 
    @test length(neg_labs1) <= max_neg 
    @test length(neg_labs1d) <= max_neg 
    # @test length(neg_labs2) <= max_neg 
    for l in neg_sols 
        @test l.cost < 0.0
    end
    for l in neg_labs1 
        @test l.cost < 0.0
    end
    for l in neg_labs1d
        @test l.cost < 0.0
    end    
    # for l in neg_labs2 
    #     @test l.cost < 0.0
    # end    
end
