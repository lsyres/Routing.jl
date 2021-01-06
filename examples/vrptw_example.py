# Solving VRPTW 
# Modified from the example given by Google OR-Tools https://developers.google.com/optimization/routing/vrp

from julia import Routing 
import numpy as np
from math import isclose
Inf = np.Inf

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
early_time = [time_windows[i][0] for i in range(n_customers)]
late_time = [time_windows[i][1] for i in range(n_customers)]

# Service time is zero in each node in this example
service_time = np.zeros(n_customers)

# Customer demand, only for customer nodes
load = [1, 1, 2, 4, 2, 4, 8, 8, 1, 2, 1, 2, 4, 4, 8, 8]

# Create a set of requests
requests = []
for i in range(n_customers):
    request_id = i + 1
    requests.append(Routing.Request(request_id, early_time[i], late_time[i], load[i], service_time[i]))




# Create the fleet information
n_vehicles = n_customers    # assume there are enough number of vehicles
capacity = 15               # vehicle capacity
max_travel_time = 35        # the latest time that each vehicle returns to the depot
fleet = Routing.Fleet(n_vehicles, capacity, max_travel_time)



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
nodes = []
for i in range(n_customers+1):
    node_id = i
    cx = coordinates[i][0]
    cy = coordinates[i][1]
    nodes.append(Routing.Node(node_id, cx, cy))


# create a Solomon
solomon = Routing.Solomon("Example VRPTW", nodes, fleet, requests) 

# solve
# `digits` is the number of digits after the decimal point
# to be considered in the Euclidean cost calculation
routes, total_distance = Routing.solveVRPpy(solomon, digits=3, pricing_method="pulse")

print("total_distance = ", total_distance)
print("routes = ", routes)
assert isclose(total_distance, 55.128)