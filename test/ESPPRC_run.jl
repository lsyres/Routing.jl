include("../src/read_solomon_data.jl")
include("../src/pulse.jl")

using ElasticArrays
# using Gurobi, JuMP
using Random
# Random.seed!(0)

dataset_name = "C102_025"
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

t = calculate_cost(nodes)
t[depot0, depot_dummy] = Inf
t[depot_dummy, depot0] = Inf
for i in N, j in N 
    if i in C
        t[i, j] = t[i, j] + requests[i].service_time
    else 
        t[i, j] = t[i, j]
    end
end

origin = depot0
destination = depot_dummy
capacity = fleet.capacity
cost_mtx = t .* (rand(size(t)...) .- 0.50)
time_mtx = t

resrc_mtx = zeros(n_nodes, n_nodes)
for i in N, j in N 
    if j in C
        resrc_mtx[i, j] = requests[j].quantity
    end
end
early_time = [a; 0; 0]
late_time = [b; 0; fleet.max_travel_time]




# pulse_path = solveESPPRC(origin, destination, resrc_cap, cost_mtx, time_mtx, resrc_mtx, early_time, late_time)
# @show pulse_path.path, pulse_path.cost, pulse_path.time

pg = PulseGraph(
    origin,
    destination,
    capacity,
    cost_mtx,
    time_mtx,
    resrc_mtx,
    early_time,
    late_time,
)



##############################################################################################################
# cost_mtx[origin, destination] = 10000
# cost_mtx[destination, origin] = 10000
# time_mtx[origin, destination] = 10000
# time_mtx[destination, origin] = 10000

# bigM = maximum(b) + maximum(t)
# bigQ = q + maximum(d)

# n_vehicles = 1
# m = Model(Gurobi.Optimizer)
# @variable(m, x[N, N], Bin)
# @variable(m, s[C] >= 0)
# @variable(m, qq[N] >= 0)

# @objective(m, Min, sum(cost_mtx[i,j] * x[i,j] for i in N, j in N))

# @constraint(m, [i in N, j in C], qq[j] >= qq[i] + d[j] - bigQ * (1 - x[i,j]))
# @constraint(m, [i in N], 0 <= qq[i] <= q)

# @constraint(m, sum(x[depot0,j] for j in N) == n_vehicles )
# @constraint(m, [h in C], sum(x[i,h] for i in N) - sum(x[h,j] for j in N) == 0 )
# @constraint(m, sum(x[i,depot_dummy] for i in N) == sum(x[depot0,j] for j in N) )

# @constraint(m, [i in C, j in C], s[j] >= s[i] + time_mtx[i,j] - bigM * (1 - x[i,j]))
# @constraint(m, [i in C], a[i] <= s[i] <= b[i])

# @constraint(m, [i in N], x[i, depot0] == 0)
# @constraint(m, [i in N], x[depot_dummy, i] == 0)
# @constraint(m, [i in N], x[i, i] == 0)

# optimize!(m)
# @show raw_status(m)
# @show objective_value(m)

# xx = zeros(size(x))
# for i in 1:size(xx,1)
#     for j in 1:size(xx,2)
#         xx[i,j] = JuMP.value.(x[i,j])
#     end
# end

# function to_path(xx, origin, destination)
#     current = origin 
#     path = [current]
#     while current != destination
#         j = findall(x -> x==1, xx[current,:])
#         push!(path, j[1])
#         current = j[1]
#     end
#     return path
# end

# path = to_path(xx, origin, destination)
# @show path

##############################################################################################################





@time pulse_path = solveESPPRCpulse(pg)
@show pulse_path.path, pulse_path.cost, pulse_path.time


# @assert isapprox(objective_value(m), pulse_path.cost; atol=1e-6)
# @assert path == pulse_path.path