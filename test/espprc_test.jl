# Solving ESPPRC 
# The example given by Google OR-Tools https://developers.google.com/optimization/routing/vrp
# Modified

# using VRPTW
include("../src/VRPTWinclude.jl")
using Test 

# For testing purpose
using Random
# Random.seed!(23423) # bug instance 
# Random.seed!(1)

# For Debugging
include("debugging.jl")

solomon_dataset_name = "C102_050"
solomon = load_solomon(solomon_dataset_name)
n_customers = solomon.nodes |> length
dual_var = rand(1:10, n_customers)
pg = solomon_to_espprc(solomon, dual_var)

##################################################

@info("ESPPRC $(solomon_dataset_name) testing...")
print("Pulse     : "); @time pulse = solveESPPRC(pg, method="pulse")
print("Mono      : "); @time mono0= solveESPPRC(pg, method="monodirectional")
print("Mono DSSR : "); @time mono1 = solveESPPRC(pg, method="monodirectional", DSSR=true)
print("Bi        : "); @time bidi0= solveESPPRC(pg, method="bidirectional")
print("Bi   DSSR : "); @time bidi1 = solveESPPRC(pg, method="bidirectional", DSSR=true)

@show pulse.cost, pulse.load, pulse.time
@show mono0.cost, mono0.load, mono0.time
@show mono1.cost, mono1.load, mono1.time
@show bidi0.cost, bidi0.load, bidi0.time
@show bidi1.cost, bidi1.load, bidi1.time
@show pulse.path
@show mono0.path
@show mono1.path
@show bidi0.path
@show bidi1.path

@testset "ESPPRC $(solomon_dataset_name) Test" begin
    @test isapprox(pulse.cost, mono0.cost, atol=1e-7)
    @test isapprox(mono0.cost, mono1.cost, atol=1e-7)
    @test isapprox(mono0.cost, mono1.cost, atol=1e-7)
    @test isapprox(pulse.cost, bidi0.cost, atol=1e-7)
    @test isapprox(bidi0.cost, bidi1.cost, atol=1e-7)
end

println("done")

# show_details(sol.path, pg)
# show_details(lab1.path, pg)
# show_details(lab2.path, pg)

############################################################
max_neg = 20
@info("ESPPRC $(solomon_dataset_name) testing with max_neg_cost_routes=$(max_neg)...")

@time sol, neg_sols = solveESPPRC_vrp(pg, method="pulse", max_neg_cost_routes=max_neg)
@time lab1, neg_labs1 = solveESPPRC_vrp(pg, method="monodirectional", max_neg_cost_routes=max_neg)
@time lab1d, neg_labs1d = solveESPPRC_vrp(pg, method="monodirectional", max_neg_cost_routes=max_neg, DSSR=true)

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
