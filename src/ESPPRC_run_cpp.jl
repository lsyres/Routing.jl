using ElasticArrays
using Gurobi, JuMP
using Random
# Random.seed!(0)

fixed_rand = [3.8237089201877836,5.935211267605691,3.5788732394366782,7.671830985915506,6.599999999999994,4.678873239436655,11.300000000000072,16.204225352112584,5.099999999999973,12.976291079812267,6.776291079812243,12.121126760563307,6.208450704225527,12.900000000000034,18.743661971831017,0.6999999999999886,18.164788732394427,12.098356807511898,10.623708920187784,14.301291079812213,15.335211267605604,8.099999999999994,8.43521126760563,8.878873239436668,23.042253521126618,14.214084507042223,0.5999999999999943,0.7999999999999545,26.52887323943669,18.499999999999915,4.376291079812155,14.477582159624472,7.247417840375427,17.850000000000094,29.799999999999997,22.476291079812142,4.199999999999996,20.352112676056265,17.021126760563277,6.28591549295777,8.099999999999994,12.75633802816887,31.16478873239444,5.300000000000068,13.80000000000009,14.600000000000009,1.5237089201877154,16.66478873239444,39.23521126760568,14.007042253521107,15.048122065727835,10.67629107981211,8.70000000000001,15.235915492957815,6.135915492957793,11.999999999999936,6.1563380281688715,4.756338028168925,4.3352112676056045,5.400000000000006,12.200000000000017,6.9525821596244235,26.847417840375446,19.40000000000003,37.94553990610343,15.521126760563313,30.90000000000004,7.721126760563319,10.123708920187838,7.798708920187778,20.40704225352111,6.578873239436646,9.799999999999972,16.33591549295781,5.300000000000011,11.928873239436705,5.300000000000026,5.599999999999959,6.77887323943666,3.70000000000001,2.7230046948355415,9.499999999999911,11.20000000000001,12.956338028168986,2.752112676056356,20.487323943661863,16.48732394366207,6.399999999999917,2.3999999999999773,21.001291079812308,6.299999999999997,1.700000000000017,11.700000000000017,9.142253521126836,1.0000000000000036,8.435211267605553,4.900000000000006,11.26478873239446,6.587323943661858,3.299999999999983]


include("readdata.jl")
# include("ESPPRC.jl")

data_filename = joinpath("solomon-1987", "C102_025.xml")


dataset, data_name, nodes, fleet, requests = readdata(data_filename)
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
        t[i, j] = t[i, j]
    else 
        t[i, j] = t[i, j]
    end
end

origin = depot0
destination = depot_dummy
capacity = fleet.capacity
cost_mtx = copy(t)

# Below is how vrp-espprc c++ library modifies the problem
for i in C 
    for j in C 
        cost_mtx[i, j] -= fixed_rand[i] 
    end
    cost_mtx[i, destination] -= fixed_rand[i]
end

time_mtx = copy(t)
for i in N, j in N 
    if i in C
        time_mtx[i, j] = time_mtx[i, j] + requests[i].service_time
    else 
        time_mtx[i, j] = time_mtx[i, j]
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




# pulse_path = solveESPPRC(origin, destination, resrc_cap, cost_mtx, time_mtx, resrc_mtx, early_time, late_time)
# @show pulse_path.path, pulse_path.cost, pulse_path.time

include("pulse.jl")
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




# function show_path_cost_detail(path, pg)
#     total_cost = 0
#     for i in 1:length(path)-1
#         n1 = path[i]
#         n2 = path[i+1]
#         println("From $(n1) to $(n2): $(pg.cost[n1, n2])")
#         total_cost += pg.cost[n1, n2]
#     end
#     println("--- total cost = $(total_cost)")
# end

# show_path_cost_detail(pulse_path.path, pg)

# cpp_path = [26 20 21 25 8 10 11 9 6 4 7 3 5 27]
# show_path_cost_detail(cpp_path, pg)




# @assert isapprox(objective_value(m), pulse_path.cost; atol=1e-6)
# @assert path == pulse_path.path