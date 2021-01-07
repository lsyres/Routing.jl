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



coordinates = [
    (4.56, 3.20),   # location  0 - the depot
    (2.28, 0.00),   # location  1
    (9.12, 0.00),   # location  2
    (0.00, 0.80),   # location  3
    (1.14, 8.00),   # location  4
    (5.70, 1.60),   # location  5
    (7.98, 1.60),   # location  6
    (3.42, 2.40),   # location  7
    (6.84, 2.40),   # location  8
    (5.70, 4.00),   # location  9
    (9.12, 4.00),   # location 10
    (1.14, 4.80),   # location 11
    (2.28, 4.80),   # location 12
    (3.42, 5.60),   # location 13
    (6.84, 5.60),   # location 14
    (0.00, 6.40),   # location 15
    (7.98, 6.40)    # location 16
]
# Create a set of nodes 
nodes = Vector{Node}()
for i in 0:n_customers
    node_id = i
    cx = coordinates[i+1][1]
    cy = coordinates[i+1][2]
    push!(nodes, Node(node_id, cx, cy))
end

# create a Solomon
solomon = Solomon(nodes, fleet, requests) 

# solve
# `digits` is the number of digits after the decimal point
# to be considered in the Euclidean cost calculation
@time routes, total_distance = solveVRP(solomon, digits=3, pricing_method="pulse")

@show total_distance
@show routes
@test isapprox(total_distance, 55.128)