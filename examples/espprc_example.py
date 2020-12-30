import numpy as np

# This example is modifed from the VRPTW example from OR-Tools
# But ESPPRC can be used outside the VRP context
# This example describes non-VRP context

# Number of nodes
n_nodes = 17

# cost matrix:
# The objective of ESPPRC is to minimize the total cost 
# Randomly generated in this example 
# cost allows negative values
cost = - np.random.rand(n_nodes, n_nodes)

# travel_time matrix:
# Some elements are Inf, meaning disconnected
# travel_time is used to check the time windows constraints
travel_time = np.matrix([
    [0, 8, 3, 2, 6, 8, 4, 8, 8, 13, 7, 5, 8, 12, 10, np.Inf, 6],  
    [8, 0, 11, 10, 6, 3, 9, 5, 8, 4, 15, 14, 13, 9, 18, 9, 9],  
    [3, 11, 0, 1, 7, 10, 6, 10, 10, 14, 6, 7, 9, 14, 6, 16, 8],  
    [2, 10, 1, 0, 6, 9, 4, 8, 9, 13, 4, 6, 8, 12, 8, 14, 7],  
    [6, 6, 7, 6, 0, 2, 3, 2, 2, 7, 9, 7, 7, 6, 12, 8, 3],  
    [8, 3, 10, 9, 2, 0, 6, 2, 5, 4, np.Inf, 10, 10, 6, np.Inf, 5, 6],  
    [4, 9, 6, 4, 3, 6, 0, 4, 4, 8, 5, 4, 3, 7, 8, 10, 2],  
    [8, 5, 10, 8, 2, 2, 4, 0, 3, 4, 9, 8, 7, 3, 13, 6, 3],  
    [8, 8, 10, 9, 2, 5, 4, 3, 0, 4, 6, 5, 4, 3, 9, 5, 2],  
    [13, 4, 14, 13, 7, 4, 8, 4, 4, 0, 10, 9, 8, 4, 13, 4, 6],  
    [7, np.Inf, 6, 4, 9, np.Inf, 5, 9, 6, 10, 0, 1, 3, 7, 3, 10, 6],  
    [5, 14, 7, 6, 7, 10, 4, 8, 5, 9, 1, 0, 2, 6, 4, 8, 4],  
    [8, 13, 9, 8, 7, 10, 3, 7, 4, 8, 3, 2, 0, 4, 5, 6, 4],  
    [12, 9, 14, 12, 6, 6, 7, 3, 3, 4, 7, 6, 4, 0, 9, 2, 5],  
    [10, 18, 6, 8, 12, 15, 8, 13, 9, 13, 3, 4, 5, 9, 0, 9, 9],  
    [np.Inf, 9, 16, 14, 8, 5, 10, 6, 5, 4, 10, 8, 6, 2, 9, 0, 7],  
    [6, 9, 8, 7, 3, 6, 2, 3, 2, 6, 6, 4, 4, 5, 9, 7, 0]
])

# load matrix 
# The load information is for each arc, hence given as a matrix
# If you have load on each node, then it should be converted to a matrix
# For example, load[i, j] = load_on_node[j] 
load_on_node = [1, 1, 2, 4, 2, 4, 8, 8, 1, 2, 1, 2, 4, 4, 8, 8, 0]
load = np.zeros([n_nodes, n_nodes])
for i in range(n_nodes):
    for j in range(n_nodes):
        load[i][j] = load_on_node[j]

# Time windows, for all nodes, including origin and destination
time_windows = [
    (0, 12), 
    (0, 15), 
    (16, 28),
    (10, 13),
    (0, 5),  
    (5, 10), 
    (0, 4),  
    (5, 10), 
    (0, 3),  
    (10, 16),
    (10, 15),
    (0, 5),  
    (5, 10), 
    (7, 8),  
    (10, 15),
    (11, 15),
    (0, 35),
]
# If early_time is set for the origin, it means the earliest possible departure time.
early_time = [time_windows[i][0] for i in range(n_nodes)]
late_time = [time_windows[i][1] for i in range(n_nodes)]

# vehicle capacity
capacity = 15 

# service_time is just set to all zeros in this example
service_time = np.zeros(n_nodes)



# IMPORTANT: to be passed on to Julia, you should add +1 to the indices
origin, destination = 5, 2
origin, destination = origin + 1, destination + 1

from julia import Routing 

pg = Routing.ESPPRC_Instance(
    origin,
    destination,
    capacity,
    cost,
    travel_time,
    load,
    early_time,
    late_time,
    service_time
)

solution_label = Routing.solveESPPRC(pg)

# IMPORTANT: Again, you should subtract 1 to the indices in `path`. 
print(solution_label.path - 1)
print(solution_label.time)
print(solution_label.cost)
print(solution_label.load)