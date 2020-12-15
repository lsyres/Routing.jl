using JuMP, GLPK, Gurobi, CPLEX
using ElasticArrays
using DelimitedFiles

include("readdata.jl")
include("pulse.jl")

debug = false

function solve_cg(filename)

    dataset, data_name, node, fleet, request = readdata(data_filename)
    n_nodes = length(node)
    n_vehicles = fleet.number
    n_requests = length(request)

    # Add a dummy node for depot at the end
    push!(node, node[n_nodes])
    depot0 = n_requests + 1
    depot_dummy = n_requests + 2
    cost = calculate_cost(node)
    cost[depot0, depot_dummy] = Inf
    cost[depot_dummy, depot0] = Inf

    n_nodes = length(node)
    @assert n_nodes - 2 == n_requests
    V = 1:n_vehicles
    N = 1:n_nodes
    C = 1:n_requests

    d = [request[i].quantity for i in C]
    q = fleet.capacity
    a = [request[i].start_time for i in C]
    b = [request[i].end_time for i in C]
    t = copy(cost)
    θ = [request[i].service_time for i in C]
    for i in N, j in N 
        if i in C
            t[i, j] = cost[i, j] + θ[i]
        else 
            t[i, j] = cost[i, j]
        end
    end


    # Prepare data for ESPPRC 
    # solveESPPRC(origin, destination, resrc_cap, cost_mtx, resrc_mtx, tw_mtx)
    origin = depot0
    destination = depot_dummy
    capacity = fleet.capacity
    cost_mtx_org = t
    resrc_mtx = zeros(n_nodes, n_nodes)
    for i in N, j in N 
        if j in C
            resrc_mtx[i, j] = request[j].quantity
        end
    end
    early_time = [a; 0; 0]
    late_time = [b; 0; fleet.max_travel_time]
    service_time = [θ; 0; 0]
    load = [d; 0; 0]

    function calcuate_cost_routes(new_route)
        route_cost = 0.0
        for i in 1:length(new_route)-1
            route_cost += cost_mtx_org[new_route[i], new_route[i+1]]
        end
        return route_cost
    end

    function add_route!(routes, cost_route, incidence, new_route)
        push!(routes, new_route)
        push!(cost_route, calcuate_cost_routes(new_route))
        customers = new_route[2:end-1]
        inc = zeros(Int, n_requests)
        inc[customers] .= 1 
        append!(incidence, inc)    
    end

    routes = []
    cost_route = []
    incidence = ElasticArray{Float64}(undef, n_requests, 0)
    for r in request
        new_route = [depot0, r.node, depot_dummy]
        add_route!(routes, cost_route, incidence, new_route)
        # push!(routes, new_route)
        # push!(cost_route, calcuate_cost_routes(new_route))
        # inc = zeros(Int, n_requests)
        # inc[r.node] = 1 
        # append!(incidence, inc)
    end

    sol_routes = []
    sol_y = []
    max_iter = 10000

    # gurobi_env = Gurobi.Env()

    old_dual_var = []
    for iter in 1:max_iter
        # RMP = Model(Gurobi.Optimizer)
        RMP = Model(optimizer_with_attributes(CPLEX.Optimizer, "CPX_PARAM_SCRIND" => 0))
        set_R = 1:length(routes)
        set_C = 1:n_requests
        @variable(RMP, y[set_R] >= 0)
        @objective(RMP, Min, sum(cost_route[r] * y[r] for r in set_R))
        @constraint(RMP, rrc[i in set_C], sum(incidence[i, r] * y[r] for r in set_R) == 1)
        optimize!(RMP)

        # Initialize the dual variable α   
        # α[depot0] is for the depot
        # depot0 is the origin, depot_dummy is the destination
        dual_var = dual.(rrc)        
        α = [dual_var; 0.0; 0.0]
        !debug || @show α
        !debug || @show objective_value(RMP)

        
        @info("---- iteration $iter -----(n_routes = $(length(routes))-----(obj = $(objective_value(RMP)))---------------")

        # Create a new cost_mtx 
        cost_mtx = copy(cost_mtx_org)
        for i in N, j in N 
            if i in C
                cost_mtx[i, j] -= α[i]
            end
        end
        time_mtx = copy(cost_mtx_org)

        # solve ESPPRC

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

        !debug || println("Solving ESPPRC---------------")
        dp_state = solveESPPRCpulse(pg)

        !debug || @show iter, dp_state
        !debug || @show in(dp_state.path, routes)
        if ! in(dp_state.path, routes) && dp_state.cost < - 1e-6
            add_route!(routes, cost_route, incidence, dp_state.path)
            sol_y = JuMP.value.(y)
            sol_routes = routes   
        else 
            println("--- no new route is added. ---")
            sol_y = JuMP.value.(y)
            sol_routes = routes           
            if !isapprox(dp_state.cost, 0, atol=1e-6)        
                @warn("The reduced_cost is negative: ", dp_state.cost)
            end
            break
        end
        println("New path: ", dp_state.path)
        println("Reduced cost: ", dp_state.cost)

    end

    for n in 1:length(sol_y)
        if sol_y[n] > 0.01
            @show n, sol_y[n], sol_routes[n]
        end
    end

end


data_filename = joinpath("solomon-1987", "R101_100.xml")

@time solve_cg(data_filename)