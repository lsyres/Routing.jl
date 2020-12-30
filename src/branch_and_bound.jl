


# branch and bound


function remove_arc_in_routes!(routes, travel_time)
    routes_to_be_removed = []
    for r_idx in eachindex(routes)
        r = routes[r_idx]
        if calculate_path_cost(r, travel_time) == Inf 
            push!(routes_to_be_removed, r_idx)
        end
    end
    deleteat!(routes, routes_to_be_removed)
end

function update_travel_time!(new_vrptw, arc::Tuple{Int, Int}, value::Int)
    i, j = arc
    n_nodes = size(new_vrptw.travel_time, 1)
    depot0 = n_nodes - 1
    depot_dummy = n_nodes
    
    if value == 0
        new_vrptw.travel_time[i, j] = Inf
    elseif value == 1
        for k in 1:n_nodes
            if k != i && k != depot0 && j != depot_dummy
                new_vrptw.travel_time[k, j] = Inf
            end
            if k != j && k != depot_dummy && i != depot0
                new_vrptw.travel_time[i, k] = Inf
            end
        end
    end
end

function generate_branch_instance(arc::Tuple{Int, Int}, value::Int, current_branch)
    i, j = arc

    new_vrptw = deepcopy(current_branch.vrptw_instance)
    new_routes = deepcopy(current_branch.root_routes)

    n_nodes = size(new_vrptw.travel_time, 1)

    update_travel_time!(new_vrptw, arc, value)

    remove_arc_in_routes!(new_routes, new_vrptw.travel_time)

    return new_routes, new_vrptw
end


function select_new_arc(branching_history, branch_priority, vrptw::VRPTW_Instance)
    n_nodes = size(vrptw.travel_time, 1)

    if isempty(branch_priority)
        for i in 1:n_nodes, j in 1:n_nodes
            arc = (i, j)
            if !in(arc, keys(branching_history)) 
                return arc, branch_priority
            end
        end
    else
        new_branch_priority = copy(branch_priority)
        arc = popfirst!(new_branch_priority)[1]
        while in(arc, keys(branching_history)) 
            arc = popfirst!(new_branch_priority)[1]
        end
        return arc, new_branch_priority
    end
end



function show_current(sol_y, sol_routes)
    for n in 1:length(sol_y)
        if sol_y[n] > 0.01
            @show n, sol_y[n], sol_routes[n]
        end
    end    
end


function  branch_decision!(next_branches, best_sol, sol_y, sol_routes, sol_obj, new_branch)
    if isempty(sol_y)
        # stop. Infeasible

    elseif is_binary(sol_y)
        if sol_obj < best_sol.objective
            best_sol.y = sol_y
            best_sol.routes = sol_routes
            best_sol.objective = sol_obj
        end
        # stop. a binary feasible solution found. 
        
    elseif sol_obj < best_sol.objective
        # Fractional LP Solution. Still alive. Move on 
        push!(next_branches, new_branch)

    else 
        # stop. Fathomed. 

    end
end

function solve_BnB!(best_sol::BestIncumbent, branches; pricing_method="pulse")
    next_branches = Branch[]

    while !isempty(branches)

        # Sorting branches with the lower bound
        # The last element has the lowest bound
        sort!(branches, by= x->x.lower_bound, rev=true)
        current_branch = pop!(branches)

        # current_branch.branching_priority is updated here
        arc, new_branch_priority = select_new_arc(current_branch.history, current_branch.branch_priority, current_branch.vrptw_instance)

        sol_y, sol_routes, sol_obj = [], [], Inf
        for arc_value in 0:1

            # branching history is updated here
            new_history = copy(current_branch.history)
            new_history[arc] = arc_value

            new_routes, new_vrptw = generate_branch_instance(arc, arc_value, current_branch)
            
            veh_cond = current_branch.veh_cond

            if isempty(new_routes)
                sol_y, sol_routes, sol_obj = [], [], Inf
            else

                sol_y, sol_routes, sol_obj = solve_cg_rmp(new_vrptw, initial_routes=new_routes, veh_cond=veh_cond, tw_reduce=false, pricing_method=pricing_method)

                new_branch = Branch(new_history, new_vrptw, current_branch.root_routes, new_branch_priority, sol_obj, veh_cond)

                branch_decision!(next_branches, best_sol, sol_y, sol_routes, sol_obj, new_branch)

            end
        end

        if isempty(branches)
            branches = deepcopy(next_branches)
            next_branches = Branch[]
        end
        
    end

end


function generate_branch_priority(root_y, root_routes, vrptw)
    branch_score_dict = Dict()
    for n in eachindex(root_y)
        if root_y[n] > 0.01
            r = root_routes[n]
            for i in 1:length(r)-1
                arc = (r[i], r[i+1])
                if in(arc, keys(branch_score_dict))
                    branch_score_dict[arc] += max(1-root_y[n], root_y[n]) * vrptw.travel_time[arc[1], arc[2]]
                else
                    branch_score_dict[arc] = max(1-root_y[n], root_y[n]) * vrptw.travel_time[arc[1], arc[2]]
                end
            end
        end
    end        
    return sort(collect(branch_score_dict), by= x->x[2])
end

function initial_BnB!(best_sol, vrptw, root_y, root_routes, root_obj; pricing_method="pulse")
    branch_priority = generate_branch_priority(root_y, root_routes, vrptw)
    new_vrptw = deepcopy(vrptw)
    history = OrderedDict()

    for n in eachindex(root_y)
        if root_y[n] > 1.0 - 1e-6
            r = root_routes[n]
            for i in 1:length(r)-1
                arc = (r[i], r[i+1])
                history[arc] = 1
                update_travel_time!(new_vrptw, arc, 1)
            end
        end
    end                


    branches = []
    if round(sum(root_y)) == sum(root_y)
        veh_cond = ("<=", -1)
        push!(branches, Branch(history, new_vrptw, root_routes, branch_priority, root_obj, veh_cond))
    else
        new_routes = copy(root_routes)
        remove_arc_in_routes!(new_routes, new_vrptw.travel_time)        
        n_vehicles = floor(sum(root_y)) |> Int 
        vehicle_conditions = [("<=", n_vehicles), (">=", n_vehicles+1)]                
        for veh_cond in vehicle_conditions
            sol_y, sol_routes, sol_obj = solve_cg_rmp(new_vrptw, initial_routes=new_routes, veh_cond=veh_cond, tw_reduce=false, pricing_method=pricing_method)

            new_branch = Branch(history, new_vrptw, root_routes, branch_priority, sol_obj, veh_cond)

            branch_decision!(branches, best_sol, sol_y, sol_routes, sol_obj, new_branch)
        end

    end

    solve_BnB!(best_sol, branches)
    
end

function complete_BnB!(best_sol, vrptw, root_y, root_routes, root_obj; pricing_method="pulse")
    # Most Priority is at the end.
    branch_priority = generate_branch_priority(root_y, root_routes, vrptw)

    branches = []
    if round(sum(root_y)) == sum(root_y)
        veh_cond = ("<=", -1)
        history = OrderedDict()
        root_branch = Branch(history, vrptw, root_routes, branch_priority, root_obj, veh_cond)
        push!(branches, root_branch)
    else
        n_vehicles = floor(sum(root_y)) |> Int 
        vehicle_conditions = [("<=", n_vehicles), (">=", n_vehicles+1)]
        for veh_cond in vehicle_conditions
            sol_y, sol_routes, sol_obj = solve_cg_rmp(vrptw, initial_routes=root_routes, veh_cond=veh_cond, tw_reduce=false, pricing_method=pricing_method)

            history = OrderedDict()
            new_branch = Branch(history, vrptw, root_routes, branch_priority, sol_obj, veh_cond)

            branch_decision!(branches, best_sol, sol_y, sol_routes, sol_obj, new_branch)
        end

    end

    solve_BnB!(best_sol, branches)

end

function solveVRP(vrptw::VRPTW_Instance; tw_reduce=true, pricing_method="pulse")
    println("Pricing method = $pricing_method")

    start_time = time()

    # Time Windows Reduction 
    if tw_reduce
        time_window_reduction!(vrptw)
        println("Time Windows Reduced. Cumulative Time: ", time() - start_time)
    end

    # initialization
    best_sol = BestIncumbent([], [], Inf)

    # solve root node 
    @info("Solving the root LP relaxation with column generation...")
    root_y, root_routes, root_obj = solve_cg_rmp(vrptw, initial_routes=[], tw_reduce=false, pricing_method=pricing_method)
    println("Root node solved. Cumulative Time: ", time() - start_time)
    @show root_obj

    @info("Root LP relaxation solution:")
    show_current(root_y, root_routes)
    println("Is the root solution binary? ", is_binary(root_y))

    if is_binary(root_y)
        best_sol.y = root_y 
        best_sol.routes = root_routes 
        best_sol.objective = root_obj
    else            

        # # Initial branching 
        # # First branching on the number of vehicles
        # # Then on arc flow, first fixing x_ij = 1 
        # # then branch-and-bound for the rest
        # initial_BnB!(best_sol, vrptw, root_y, root_routes, root_obj)

        # @info("Initial BnB is done.")
        # println("Initial BnB. Cumulative Time: ", time() - start_time)
        # @show best_sol.objective
        # show_current(best_sol.y, best_sol.routes)


        # Complete branching 
        # First branching on the number of vehicles
        # Then on arc flow, from the beginning
        complete_BnB!(best_sol, vrptw, root_y, root_routes, root_obj)

        @info("Complete BnB is done.")
        println("Complete BnB. Cumulative Time: ", time() - start_time)
        @show best_sol.objective
        show_current(best_sol.y, best_sol.routes)

    end


    final_routes = []
    for i in eachindex(best_sol.y)
        if best_sol.y[i] > 1 - 1e-6
            push!(final_routes, best_sol.routes[i])
        end
    end
    return final_routes, best_sol.objective

    # return best_sol.y, best_sol.routes, best_sol.objective
end
