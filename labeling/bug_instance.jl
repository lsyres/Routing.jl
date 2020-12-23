# Solving ESPPRC 
# The example given by Google OR-Tools https://developers.google.com/optimization/routing/vrp
# Modified

using VRPTW

include("labeling.jl")

using Test 

# For testing purpose
using Random
Random.seed!(23423)
using ElasticArrays

dataset_name = "C102_100"

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

# alpha = rand(0:20, n_nodes) * 2
# cost_mtx = t - repeat(alpha, 1, n_nodes)
cost_mtx = t .* (rand(size(t)...) .- 0.25)

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



function show_details(path, pg::ESPPRC_Instance)
    println("--------------Path Details ----------------------")
    println(path)
    j = path[1]
    arr_time = 0.0
    load = 0.0
    cost = 0.0
    cost_change = 0.0
    println("At node $j: time=$(round(arr_time)), load=$(round(load)), cost=$(round(cost)), cost_change=$cost_change")
    for k in 2:length(path)
        i, j = path[k-1], path[k]
        arr_time = max(arr_time + pg.service_time[i] + pg.time[i,j], pg.early_time[j])
        load += pg.load[i,j]
        cost += pg.cost[i,j]
        cost_change = pg.cost[i,j]
        println("At node $j: time=$(round(arr_time)), load=$(round(load)), cost=$(round(cost)), cost_change=$cost_change")    
    end      
    println("-"^50)  
end


############################################################

@time sol = solveESPPRCpulse(ei)
@time lab1, labelset1 = monodirectional(ei)
@time lab2, labelset2 = bidirectional(ei)


@show sol.cost, sol.load, sol.time
@show lab1.cost, lab1.load, lab1.time
@show lab2.cost, lab2.load, lab2.time
@show sol.path
@show lab1.path
@show lab2.path

println("done")

show_details(sol.path, ei)
show_details(lab1.path, ei)
show_details(lab2.path, ei)


@assert isapprox(sol.cost, lab1.cost, atol=1e-7)
@assert isapprox(lab1.cost, lab2.cost, atol=1e-7)
