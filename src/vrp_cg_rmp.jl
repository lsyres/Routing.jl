

function time_window_reduction!(vrptw::VRPTW_Instance)
    n_nodes, n_customers, depot0, depot_dummy = read_vrptw_instance(vrptw::VRPTW_Instance)
    for iter in 1:3
        for k in 1:n_customers 
            # minimal arrival time from predecessors:
            min_arr_time_pred = minimum( [vrptw.early_time[i] + vrptw.travel_time[i, k] for i in 1:n_customers] )
            vrptw.early_time[k] = max(vrptw.early_time[k], min(vrptw.late_time[k], min_arr_time_pred))

            # minimal arrival time to successors:
            min_arr_time_succ = minimum( [vrptw.early_time[j] - vrptw.travel_time[k, j] for j in 1:n_customers] )
            vrptw.early_time[k] = max(vrptw.early_time[k], min(vrptw.late_time[k], min_arr_time_succ))

            # maximal departure time from predecessors:
            max_dep_time_pred = maximum( [
                vrptw.late_time[i] + vrptw.travel_time[i, k] 
                for i in 1:n_customers if vrptw.travel_time[i, k]<Inf
            ] )
            vrptw.late_time[k] = min(vrptw.late_time[k], max(vrptw.early_time[k], max_dep_time_pred))

            # maximal departure time to successors:
            max_dep_time_succ = maximum( [
                vrptw.late_time[j] - vrptw.travel_time[k, j]
                for j in 1:n_customers if vrptw.travel_time[k, j]<Inf
            ] )
            vrptw.late_time[k] = min(vrptw.late_time[k], max(vrptw.early_time[k], max_dep_time_succ))
        end
    end
end


function read_vrptw_instance(vrptw::VRPTW_Instance)
    @assert size(vrptw.travel_time, 1) == size(vrptw.travel_time, 2)
    n_nodes = size(vrptw.travel_time, 1)

    @assert length(vrptw.service_time) == length(vrptw.early_time) == length(vrptw.late_time) == length(vrptw.load)
    n_customers = length(vrptw.service_time)

    @assert n_nodes == n_customers + 2

    depot0 = n_customers + 1
    depot_dummy = n_customers + 2
    @assert vrptw.travel_time[depot0, depot_dummy] == Inf
    @assert vrptw.travel_time[depot_dummy, depot0] == Inf

    return n_nodes, n_customers, depot0, depot_dummy
end



function solve_cg_rmp(vrptw::VRPTW_Instance; optimizer = GLPK.Optimizer, initial_routes=[], tw_reduce=true)
    if tw_reduce 
        time_window_reduction!(vrptw)
    end

    n_nodes, n_customers, depot0, depot_dummy = read_vrptw_instance(vrptw)
    set_N = 1:n_nodes
    set_C = 1:n_customers

    # add service time to the cost matrix
    cost_mtx = copy(vrptw.travel_time)

    time_mtx = copy(vrptw.travel_time)
    for i in set_N, j in set_N
        if i in set_C
            time_mtx[i, j] = time_mtx[i, j] + vrptw.service_time[i]
        else 
            time_mtx[i, j] = time_mtx[i, j]
        end
    end

    origin = depot0
    destination = depot_dummy
    capacity = vrptw.capacity

    resrc_mtx = zeros(n_nodes, n_nodes)
    for i in set_N, j in set_N
        if j in set_C
            resrc_mtx[i, j] = vrptw.load[j]
        end
    end

    early_time = [vrptw.early_time; 0; 0]
    late_time = [vrptw.late_time; 0; vrptw.max_travel_time]
    service_time = [vrptw.service_time; 0; 0]
    load = [vrptw.load; 0; 0]



    function calcuate_cost_routes(new_route)
        route_cost = 0.0
        for i in 1:length(new_route)-1
            route_cost += cost_mtx[new_route[i], new_route[i+1]]
        end
        return route_cost
    end

    function add_route!(routes, cost_route, incidence, new_route)
        push!(routes, new_route)
        push!(cost_route, calcuate_cost_routes(new_route))
        customers = new_route[2:end-1]
        inc = zeros(Int, n_customers)
        inc[customers] .= 1 
        append!(incidence, inc)    
    end

    routes = []
    cost_routes = []
    incidence = ElasticArray{Float64}(undef, n_customers, 0)
    if isempty(initial_routes)
        for c in set_C
            new_route = [depot0, c, depot_dummy]
            add_route!(routes, cost_routes, incidence, new_route)
        end
    else
        for r in initial_routes
            add_route!(routes, cost_routes, incidence, r)
        end
    end

    sol_routes = []
    sol_y = []
    sol_obj = Inf
    max_iter = 10000

    # gurobi_env = Gurobi.Env()

    function solve_RMP(routes, cost_routes, incidence; optimizer=GLPK.Optimizer)
        RMP = Model(GLPK.Optimizer)
        set_R = 1:length(routes)
        set_C = 1:size(incidence, 1)
        @variable(RMP, y[set_R] >= 0)
        @objective(RMP, Min, sum(cost_routes[r] * y[r] for r in set_R))
        @constraint(RMP, rrc[i in set_C], sum(incidence[i, r] * y[r] for r in set_R) == 1)
        optimize!(RMP)

        # @show primal_status(RMP), dual_status(RMP)
        # (MathOptInterface.FEASIBLE_POINT, MathOptInterface.FEASIBLE_POINT)
        # (MathOptInterface.NO_SOLUTION, MathOptInterface.INFEASIBILITY_CERTIFICATE)

        primal_feasible = primal_status(RMP) == JuMP.MOI.FEASIBLE_POINT
        dual_feasible = dual_status(RMP) == JuMP.MOI.FEASIBLE_POINT
        is_optimal = primal_feasible &&  dual_feasible

        return JuMP.dual.(rrc), JuMP.value.(y).data, objective_value(RMP), is_optimal
    end


    for iter in 1:max_iter        
        dual_var, sol_y, sol_obj, is_optimal = solve_RMP(routes, cost_routes, incidence; optimizer=optimizer)
        
        if !is_optimal 
            return [], [], Inf
        end

        # α[depot0] is for the depot
        # depot0 is the origin, depot_dummy is the destination
        α = [dual_var; 0.0; 0.0]
        
        # @info("---- iteration $iter -----(n_routes = $(length(routes))-----(obj = $sol_obj)---------------")

        # Create a new cost_mtx 
        cost_mtx_copy = copy(cost_mtx)
        for i in set_N, j in set_N 
            if i in set_C
                cost_mtx_copy[i, j] -= α[i]
            end
        end
        time_mtx = copy(time_mtx)

        # solve ESPPRC

        pg = PulseGraph(
            origin,
            destination,
            capacity,
            cost_mtx_copy,
            time_mtx,
            resrc_mtx,
            early_time,
            late_time,
        )

        dp_state = solveESPPRCpulse(pg)

        if dp_state.path == [] || dp_state.cost == Inf
            println("--- There is no feasible path. ---")
            # @show dp_state.path, dp_state.cost
            sol_y = [] 
            sol_routes = []
            sol_obj = Inf
            break
        elseif ! in(dp_state.path, routes) && dp_state.cost < - 1e-6
            add_route!(routes, cost_routes, incidence, dp_state.path)
            sol_routes = routes   
        else 
            # println("--- no new route is added. ---")
            sol_routes = routes           
            if dp_state.cost < - 1e-6
                @warn("No new route is added, but the reduced_cost is negative: ", dp_state.cost)
            end
            break
        end
        # println("New path: ", dp_state.path)
        # println("Reduced cost: ", dp_state.cost)

    end


    sol_y = Array(sol_y)

    return sol_y, sol_routes, sol_obj

end

