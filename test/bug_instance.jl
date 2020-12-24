# Solving ESPPRC 
# The example given by Google OR-Tools https://developers.google.com/optimization/routing/vrp
# Modified


# Random.seed!(34534)
# dataset_name = "C102_025"

# Random.seed!(23423)
# dataset_name = "C102_100"


# using VRPTW
include("../src/VRPTWinclude.jl")
include("espprc_test_functions.jl")

using Test 

# For testing purpose
using Random
Random.seed!(23423); dataset_name = "C102_100"

# Random.seed!(34534); dataset_name = "C102_025"

data_file_path = dataset_name * ".xml"
data_file_path = joinpath(@__DIR__, "..", "solomon-1987", data_file_path)
dataset, data_name, nodes, fleet, requests = read_solomon_data(data_file_path)

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

# alpha = rand(0:20, n_nodes)
# cost_mtx = t - repeat(alpha, 1, n_nodes)
cost_mtx = t .* (rand(size(t)...) .- 0.25)
# cost_mtx = t

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

##################################################

############################################################

@time sol = solveESPPRC(ei, method="pulse")
# @time lab1 = solveESPPRC(ei, method="monodirectional")
@time lab2 = solveESPPRC(ei, method="bidirectional")


@show sol.cost, sol.load, sol.time
# @show lab1.cost, lab1.load, lab1.time
@show lab2.cost, lab2.load, lab2.time
@show sol.path
# @show lab1.path
@show lab2.path

println("done")

# show_details(sol.path, ei)
# show_details(lab1.path, ei)
# show_details(lab2.path, ei)


# Random.seed!(23423)
# dataset_name = "C102_100"
# [101, 3, 54, 94, 38, 1, 59, 91, 34, 102]
# -91.25575649162371, 140.0
# @show cost = -91.25575649162371
# @show load = 140.0
# @show path = [101, 3, 54, 94, 38, 1, 59, 91, 34, 102]
# @test isapprox(cost, lab1.cost, atol=1e-7)

@test isapprox(sol.cost, lab1.cost, atol=1e-7)
@test isapprox(lab1.cost, lab2.cost, atol=1e-7)
