

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



function solve_cg_rmp(vrptw::VRPTW_Instance; initial_routes=[], veh_cond=("<=",-1), tw_reduce=true, pricing_method="monodirectional")
    if tw_reduce 
        time_window_reduction!(vrptw)
    end

    n_nodes, n_customers, depot0, depot_dummy = read_vrptw_instance(vrptw)
    set_N = 1:n_nodes
    set_C = 1:n_customers

    # n_vehicles is not given in the beginning
    # later will be given for branching
    if veh_cond[2] == -1
        veh_cond = ("<=", n_customers)
    end

    # cost_mtx/time_mtx only accounts the travel time, not service time
    cost_mtx = copy(vrptw.travel_time)
    time_mtx = copy(vrptw.travel_time)

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

    function solve_RMP(routes, cost_routes, incidence, veh_cond)
        RMP = Model(GLPK.Optimizer)
        set_R = 1:length(routes)
        set_C = 1:size(incidence, 1)

        @variable(RMP, y[set_R] >= 0)
        @objective(RMP, Min, sum(cost_routes[r] * y[r] for r in set_R))
        @constraint(RMP, rrc[i in set_C], sum(incidence[i, r] * y[r] for r in set_R) == 1)

        if veh_cond[1] == "<="
            @constraint(RMP, num_vec, sum(y[r] for r in set_R) <= veh_cond[2])
        elseif veh_cond[1] == ">="
            @constraint(RMP, num_vec, sum(y[r] for r in set_R) >= veh_cond[2])
        else
            @error("Vehicle conditions should be either '<=' or '>='.")
        end
        optimize!(RMP)

        # @show primal_status(RMP), dual_status(RMP)
        # (MathOptInterface.FEASIBLE_POINT, MathOptInterface.FEASIBLE_POINT)
        # (MathOptInterface.NO_SOLUTION, MathOptInterface.INFEASIBILITY_CERTIFICATE)

        primal_feasible = primal_status(RMP) == JuMP.MOI.FEASIBLE_POINT
        dual_feasible = dual_status(RMP) == JuMP.MOI.FEASIBLE_POINT
        is_rmp_optimal = primal_feasible &&  dual_feasible

        return JuMP.dual.(rrc), JuMP.dual(num_vec), JuMP.value.(y).data, objective_value(RMP), is_rmp_optimal
    end


    for iter in 1:max_iter        
        dual_var, dual_depot, sol_y, sol_obj, is_rmp_optimal = solve_RMP(routes, cost_routes, incidence, veh_cond)
        
        if !is_rmp_optimal 
            return [], [], Inf
        end

        # α[depot0] is for the depot
        # depot0 is the origin, depot_dummy is the destination
        α = [dual_var; dual_depot; 0.0]
        
        # @info("---- iteration $iter -----(n_routes = $(length(routes))-----(obj = $sol_obj)---------------")

        # Create a new cost_mtx 
        reduced_cost_mtx = copy(cost_mtx)
        for i in set_N, j in set_N 
            if i in set_N
                reduced_cost_mtx[i, j] -= α[i]
            end
        end
        time_mtx = copy(time_mtx)

        # solve ESPPRC

        pg = ESPPRC_Instance(
            origin,
            destination,
            capacity,
            reduced_cost_mtx,
            time_mtx,
            resrc_mtx,
            early_time,
            late_time,
            service_time
        )

        t1 = time()
        best_p, all_negative_reduced_cost_paths = solveESPPRC(pg, max_neg_cost_routes=400, method=pricing_method)
        t2 = time()


        if iter == 1 || iter % 10 == 0 || abs(best_p.cost) < 1e-6
            pricing_time = string(round((t2-t1)*1000)/1000) * " s" 
            best_reduced_cost = round((best_p.cost)*1000)/1000
            n_neg_cost_paths = length(all_negative_reduced_cost_paths)
            @show iter, best_reduced_cost, n_neg_cost_paths, pricing_time
        end


        added_path_counter = 0

        if best_p.path == [] || best_p.cost == Inf
            println("--- There is no feasible path. ---")
            # @show dp_state.path, dp_state.cost
            sol_y = [] 
            sol_routes = []
            sol_obj = Inf
            break
        elseif !isempty(all_negative_reduced_cost_paths)
            is_new_route_generated = false
            for neg_p in all_negative_reduced_cost_paths
                if ! in(neg_p.path, routes) 
                    added_path_counter += 1
                    # @assert neg_p.cost < 0 
                    add_route!(routes, cost_routes, incidence, neg_p.path)
                    sol_routes = routes   
                    is_new_route_generated = true
                end
            end

            # println("-- # new added paths: $(added_path_counter)")

            if !is_new_route_generated
                sol_routes = routes      
                # @warn("No new route is added, but there are some routes with negative reduced cost.")
                # In this case, it must be a very small negative value -7.105427357601002e-15
                break
            end
            
        else
            sol_routes = routes
            # If pulse limits the number of negative reduced cost paths
            # the assertion below is not true.
            # @assert best_p.cost >= 0.0
            break
        end

    end


    println("-- Total columns = $(length(sol_routes))")
    sol_y = Array(sol_y)

    return sol_y, sol_routes, sol_obj

end

