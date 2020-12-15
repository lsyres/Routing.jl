using JuMP, GLPK
using ElasticArrays
using DelimitedFiles

include("readdata.jl")
include("pulse.jl")

debug = false

function solve_cg(filename)

    dataset, data_name, nodes, fleet, requests = readdata(data_filename)
    n_nodes = length(nodes)
    n_vehicles = fleet.number
    n_requests = length(requests)

    # Add a dummy node for depot at the end
    push!(nodes, nodes[n_nodes])
    depot0 = n_requests + 1
    depot_dummy = n_requests + 2
    cost = calculate_cost(nodes)
    cost[depot0, depot_dummy] = Inf
    cost[depot_dummy, depot0] = Inf

    n_nodes = length(nodes)
    @assert n_nodes - 2 == n_requests
    V = 1:n_vehicles
    N = 1:n_nodes
    C = 1:n_requests

    d = [requests[i].quantity for i in C]
    q = fleet.capacity
    a = [requests[i].start_time for i in C]
    b = [requests[i].end_time for i in C]
    t = copy(cost)
    θ = [requests[i].service_time for i in C]
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
            resrc_mtx[i, j] = requests[j].quantity
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
    cost_routes = []
    incidence = ElasticArray{Float64}(undef, n_requests, 0)
    for r in requests
        new_route = [depot0, r.node, depot_dummy]
        add_route!(routes, cost_routes, incidence, new_route)
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

    function solve_RMP(routes, cost_routes, requests; optimizer=GLPK.Optimizer)
        RMP = Model(GLPK.Optimizer)
        set_R = 1:length(routes)
        set_C = 1:length(requests)
        @variable(RMP, y[set_R] >= 0)
        @objective(RMP, Min, sum(cost_routes[r] * y[r] for r in set_R))
        @constraint(RMP, rrc[i in set_C], sum(incidence[i, r] * y[r] for r in set_R) == 1)
        optimize!(RMP)
        return JuMP.dual.(rrc), JuMP.value.(y), objective_value(RMP)
    end

    old_dual_var = []
    # RMP = Model(Gurobi.Optimizer)
    # RMP = Model(optimizer_with_attributes(CPLEX.Optimizer, "CPX_PARAM_SCRIND" => 0))
    optimizer = GLPK.Optimizer

    for iter in 1:max_iter        
        dual_var, y, obj_rmp = solve_RMP(routes, cost_routes, requests; optimizer=optimizer)
        # α[depot0] is for the depot
        # depot0 is the origin, depot_dummy is the destination
        α = [dual_var; 0.0; 0.0]
        !debug || @show α
        !debug || @show obj_rmp

        
        @info("---- iteration $iter -----(n_routes = $(length(routes))-----(obj = $obj_rmp)---------------")

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
            add_route!(routes, cost_routes, incidence, dp_state.path)
            sol_y = y
            sol_routes = routes   
        else 
            println("--- no new route is added. ---")
            sol_y = y
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