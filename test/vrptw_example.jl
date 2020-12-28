# Solving VRPTW 
# The example given by Google OR-Tools https://developers.google.com/optimization/routing/vrp

using VRPTW
# include("../src/VRPTWinclude.jl")

using Test 

# Given travel time matrix.
# Nodes 1--16 are customer nodes, and node 17 is the depot
travel_time = Float64[
    0 8 3 2 6 8 4 8 8 13 7 5 8 12 10 14 6; 
    8 0 11 10 6 3 9 5 8 4 15 14 13 9 18 9 9; 
    3 11 0 1 7 10 6 10 10 14 6 7 9 14 6 16 8; 
    2 10 1 0 6 9 4 8 9 13 4 6 8 12 8 14 7; 
    6 6 7 6 0 2 3 2 2 7 9 7 7 6 12 8 3; 
    8 3 10 9 2 0 6 2 5 4 12 10 10 6 15 5 6; 
    4 9 6 4 3 6 0 4 4 8 5 4 3 7 8 10 2; 
    8 5 10 8 2 2 4 0 3 4 9 8 7 3 13 6 3; 
    8 8 10 9 2 5 4 3 0 4 6 5 4 3 9 5 2; 
    13 4 14 13 7 4 8 4 4 0 10 9 8 4 13 4 6; 
    7 15 6 4 9 12 5 9 6 10 0 1 3 7 3 10 6; 
    5 14 7 6 7 10 4 8 5 9 1 0 2 6 4 8 4; 
    8 13 9 8 7 10 3 7 4 8 3 2 0 4 5 6 4; 
    12 9 14 12 6 6 7 3 3 4 7 6 4 0 9 2 5; 
    10 18 6 8 12 15 8 13 9 13 3 4 5 9 0 9 9; 
    14 9 16 14 8 5 10 6 5 4 10 8 6 2 9 0 7; 
    6 9 8 7 3 6 2 3 2 6 6 4 4 5 9 7 0
]

# Adding a dummy node for the depot
travel_time = [
    travel_time          travel_time[:, 17];
    travel_time[17, :]'                   0
]
depot0 = 17
depot_dummy = 18 
travel_time[depot0, depot_dummy] = Inf
travel_time[depot_dummy, depot0] = Inf
# travel_time is now a 18x18 matrix

# Time windows, only for customer nodes
time_windows = [
    (7, 12),  # 1
    (10, 15),  # 2
    (16, 18),  # 3
    (10, 13),  # 4
    (0, 5),  # 5
    (5, 10),  # 6
    (0, 4),  # 7
    (5, 10),  # 8
    (0, 3),  # 9
    (10, 16),  # 10
    (10, 15),  # 11
    (0, 5),  # 12
    (5, 10),  # 13
    (7, 8),  # 14
    (10, 15),  # 15
    (11, 15),  # 16
]
early_time = [time_windows[i][1] for i in 1:16]
late_time = [time_windows[i][2] for i in 1:16]

# Service time is zero in each node in this example
service_time = zeros(16)

# Customer demand, only for customer nodes
load = [1, 1, 2, 4, 2, 4, 8, 8, 1, 2, 1, 2, 4, 4, 8, 8]

# vehicle capacity
capacity = 15 

# max_travel_time is the latest time that each vehicle returns to the depot
max_travel_time = 35

# create a VRPTW instance
vrptw = VRPTW_Instance(
    travel_time,
    service_time,
    early_time,
    late_time,
    load,
    capacity,
    max_travel_time
)

# solve
@time routes, total_distance = solve_vrp_bnb(vrptw, pricing_method="pulse")

@show total_distance
@show routes


function check_route(r)
    println("------- Route: $r -------------------------------------------------")
    arrival_time = 0 
    total_load = 0
    total_dist = 0
    for k in 1:length(r)-1
        i, j = r[k], r[k+1]
        if j == depot_dummy
            tw = (0, max_travel_time)
            arrival_time = arrival_time + travel_time[i,j]
            arrival_time = max(tw[1], arrival_time)
            total_dist += travel_time[i, depot_dummy]
        else
            tw = time_windows[j]
            arrival_time = arrival_time + travel_time[i,j]
            arrival_time = max(tw[1], arrival_time)
            total_load += load[j]            
            total_dist += travel_time[i,j]
        end

        println("[Node $j] arrival_time: $(arrival_time) ∈ $(tw), total_load: $(total_load) <= $capacity, total_dist=$(total_dist)")
        @assert arrival_time <= tw[2]
        @assert total_load <= capacity
    end
    println("-------------------------------------------------------------------------------------------")

    return total_dist
end

for m in 1:length(routes)
    r = routes[m]
    total_cost = 0
    total_cost += check_route(r)
    @show total_cost
end

println("Nodes 17 and 18 are the depot.")

# Optimal Solution
total_opt_cost = 0
total_opt_cost += check_route([17, 9, 14, 16, 10, 18])
total_opt_cost += check_route([17, 7, 1, 4, 3, 18])
total_opt_cost += check_route([17, 12, 13, 15, 11, 18])
total_opt_cost += check_route([17, 5, 8, 6, 2, 18])
@show total_opt_cost

@testset "OR-Tools-Example" begin
    @test total_distance == 73.0
end