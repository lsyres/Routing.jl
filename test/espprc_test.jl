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

function solomon_to_espprc(solomon::SolomonDataset, dual_var)
    nodes, fleet, requests = solomon.nodes, solomon.fleet, solomon.requests
    n_nodes = length(nodes)
    n_vehicles = fleet.number
    n_requests = length(requests)

    # Add a dummy node for depot at the end
    push!(nodes, nodes[n_nodes])
    depot0 = n_requests + 1
    depot_dummy = n_requests + 2

    n_nodes = length(nodes)
    @assert n_nodes - 2 == n_requests
    V = 1:n_vehicles
    N = 1:n_nodes
    C = 1:n_requests

    d = [requests[i].quantity for i in C]
    q = fleet.capacity
    a = [requests[i].start_time for i in C]
    b = [requests[i].end_time for i in C]
    θ = [requests[i].service_time for i in C]

    t = calculate_solomon_cost(nodes)
    t[depot0, depot_dummy] = Inf
    t[depot_dummy, depot0] = Inf
    time_mtx = copy(t)

    origin = depot0
    destination = depot_dummy
    capacity = fleet.capacity

    # alpha = rand(0:20, n_nodes) * 2
    # cost_mtx = t - repeat(alpha, 1, n_nodes)
    # cost_mtx = t .* (rand(size(t)...) .- 0.25)
    for i in N
        for j in C
            cost_mtx[i,j] -= dual_var[i]
        end
    end

    resrc_mtx = zeros(n_nodes, n_nodes)
    for i in N, j in N 
        if j in C
            resrc_mtx[i, j] = requests[j].quantity
        end
    end
    early_time = [a; 0; 0]
    late_time = [b; 0; fleet.max_travel_time]
    service_time = [θ; 0; 0]
    travel_time = copy(t)


    ei = ESPPRC_Instance(
        origin,
        destination,
        capacity,
        cost_mtx,
        travel_time,
        resrc_mtx,
        early_time,
        late_time,
        service_time
    )
end




dataset_name = "C102_100"
solomon_dataset_name = "C101_100"
solomon = load_solomon(solomon_dataset_name)
n_nodes = solomon.nodes |> length
dual_var = (rand(n_nodes) * 20)
dual_var[1] = 0.0
espprc = solomon_to_espprc(solomon, dual_var)

##################################################



@info("ESPPRC $(dataset_name) testing...")
@time sol = solveESPPRC(ei, method="pulse")
@time lab1 = solveESPPRC(ei, method="monodirectional")
@time lab2 = solveESPPRC(ei, method="bidirectional")


@show sol.cost, sol.load, sol.time
@show lab1.cost, lab1.load, lab1.time
@show lab2.cost, lab2.load, lab2.time
@show sol.path
@show lab1.path
@show lab2.path

@testset "ESPPRC $(dataset_name) Test" begin
    @test isapprox(sol.cost, lab1.cost, atol=1e-7)
end

println("done")

show_details(sol.path, ei)
show_details(lab1.path, ei)
show_details(lab2.path, ei)

############################################################
max_neg = 20
@info("ESPPRC $(dataset_name) testing with max_neg_cost_routes=$(max_neg)...")

@time sol, neg_sols = solveESPPRC(ei, method="pulse", max_neg_cost_routes=max_neg)
@time lab1, neg_labs1 = solveESPPRC(ei, method="monodirectional", max_neg_cost_routes=max_neg)
@time lab2, neg_labs2 = solveESPPRC(ei, method="bidirectional", max_neg_cost_routes=max_neg)

@testset "ESPPRC $(dataset_name) Test with max_neg_cost_routes" begin
    @test length(neg_sols) <= max_neg 
    @test length(neg_labs1) <= max_neg 
    @test length(neg_labs2) <= max_neg 
    for l in neg_sols 
        @test l.cost < 0.0
    end
    for l in neg_labs1 
        @test l.cost < 0.0
    end
    for l in neg_labs2 
        @test l.cost < 0.0
    end    
end
