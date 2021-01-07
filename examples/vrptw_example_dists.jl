# Solving VRPTW 
# Modified from the example given by Google OR-Tools https://developers.google.com/optimization/routing/vrp

using Routing, OffsetArrays
# include("../src/Routing_include.jl")

using Test 

n_customers = 16

# Time windows, only for customer nodes
time_windows = [
    (7, 12),    # customer  1
    (10, 15),   # customer  2
    (16, 18),   # customer  3
    (10, 13),   # customer  4
    (0, 5),     # customer  5
    (5, 10),    # customer  6
    (0, 4),     # customer  7
    (5, 10),    # customer  8
    (0, 3),     # customer  9
    (10, 16),   # customer 10
    (10, 15),   # customer 11
    (0, 5),     # customer 12
    (5, 10),    # customer 13
    (7, 8),     # customer 14
    (10, 15),   # customer 15
    (11, 15),   # customer 16
]
early_time = [time_windows[i][1] for i in 1:n_customers]
late_time = [time_windows[i][2] for i in 1:n_customers]

# Service time is zero in each node in this example
service_time = zeros(n_customers)

# Customer demand, only for customer nodes
load = [1, 1, 2, 4, 2, 4, 8, 8, 1, 2, 1, 2, 4, 4, 8, 8]

# Create a set of requests
requests = Vector{Request}()
for i in 1:n_customers
    request_id = i     
    push!(requests, Request(request_id, early_time[i], late_time[i], load[i], service_time[i]))
end



# Create the fleet information
n_vehicles = n_customers    # assume there are enough number of vehicles
capacity = 15               # vehicle capacity
max_travel_time = 35        # the latest time that each vehicle returns to the depot
fleet = Fleet(n_vehicles, capacity, max_travel_time)



# Create a set of nodes 
nodes = Vector{Node}()
# if you can supply coordinates (cx, cy) to calculate the distance 
# if you don't have coordinates, just put (0.0, 0.0)
push!(nodes, Node(0, 0.0, 0.0))   # (id, cx, cy)
for i in 1:n_customers
    node_id = i
    push!(nodes, Node(node_id, 0.0, 0.0))
end

# Since we didn't supply the coordinates, we need to provide the distance information
dists_data = Float64[
    0 6 9 8 7 3 6 2 3 2 6 6 4 4 5 9 7;
    6 0 8 3 2 6 8 4 8 8 13 7 5 8 12 10 14; 
    9 8 0 11 10 6 3 9 5 8 4 15 14 13 9 18 9; 
    8 3 11 0 1 7 10 6 10 10 14 6 7 9 14 6 16; 
    7 2 10 1 0 6 9 4 8 9 13 4 6 8 12 8 14; 
    3 6 6 7 6 0 2 3 2 2 7 9 7 7 6 12 8; 
    6 8 3 10 9 2 0 6 2 5 4 12 10 10 6 15 5; 
    2 4 9 6 4 3 6 0 4 4 8 5 4 3 7 8 10; 
    3 8 5 10 8 2 2 4 0 3 4 9 8 7 3 13 6; 
    2 8 8 10 9 2 5 4 3 0 4 6 5 4 3 9 5; 
    6 13 4 14 13 7 4 8 4 4 0 10 9 8 4 13 4; 
    6 7 15 6 4 9 12 5 9 6 10 0 1 3 7 3 10; 
    4 5 14 7 6 7 10 4 8 5 9 1 0 2 6 4 8; 
    4 8 13 9 8 7 10 3 7 4 8 3 2 0 4 5 6; 
    5 12 9 14 12 6 6 7 3 3 4 7 6 4 0 9 2; 
    9 10 18 6 8 12 15 8 13 9 13 3 4 5 9 0 9; 
    7 14 9 16 14 8 5 10 6 5 4 10 8 6 2 9 0; 
]
# In the above `travel_time_data`, the index '1' means depot and '2' means customer #1, and so on.
# We want the index '0' means the depot, and '1' means customer #1.
dists_oa = OffsetArray(dists_data, 0:n_customers, 0:n_customers)

# create a Solomon
solomon = Solomon(nodes, fleet, requests) 

# solve
@time routes, total_distance = solveVRP(solomon, dists_oa, pricing_method="pulse")
# If you have given proper coordinates, don't pass `dists` as follows:
# @time routes, total_distance = solveVRP(solomon, pricing_method="pulse")

@show total_distance
@show routes
@test total_distance == 73.0