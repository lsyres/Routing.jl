


# branch and bound

function is_binary(y)
    tol = 1e-6
    for i in eachindex(y)
        if ! ( isapprox(y[i], 1.0, atol=tol) || isapprox(y[i], 0.0, atol=tol) )
            return false
        end
    end
    return true
end

function route_cost(new_route, travel_time)
    r_cost = 0.0
    for i in 1:length(new_route)-1
        r_cost += travel_time[new_route[i], new_route[i+1]]
    end
    return r_cost
end


function remove_arc_in_routes!(routes, travel_time)
    routes_to_be_removed = []
    for r_idx in eachindex(routes)
        r = routes[r_idx]
        if route_cost(r, travel_time) == Inf 
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
        arc = pop!(new_branch_priority)[1]
        while in(arc, keys(branching_history)) 
            arc = pop!(new_branch_priority)[1]
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


function solve_BnB!(best_sol::BestIncumbent, root_branch)
    branches = [root_branch]
    next_branches = Branch[]

    while !isempty(branches)
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
            
            if isempty(new_routes)
                sol_y, sol_routes, sol_obj = [], [], Inf
            else
                sol_y, sol_routes, sol_obj = solve_cg_rmp(new_vrptw, initial_routes=new_routes, tw_reduce=false)
            end

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
                push!(next_branches, Branch(new_history, new_vrptw, 
                                            current_branch.root_routes, new_branch_priority, sol_obj))

            else 
                # stop. Fathomed. 

            end
        end

        if isempty(branches)
            branches = deepcopy(next_branches)
            next_branches = Branch[]
        end
        
    end

end


function generate_branch_priority(root_y, root_routes)
    branch_score_dict = Dict()
    for n in eachindex(root_y)
        if root_y[n] > 0.01
            r = root_routes[n]
            for i in 1:length(r)-1
                arc = (r[i], r[i+1])
                if in(arc, keys(branch_score_dict))
                    branch_score_dict[arc] += root_y[n]
                else
                    branch_score_dict[arc] = root_y[n]
                end
            end
        end
    end        
    return sort(collect(branch_score_dict), by= x->x[2])
end

function initial_BnB!(best_sol, vrptw, root_y, root_routes, root_obj)
    branch_priority = generate_branch_priority(root_y, root_routes)
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

    initial_branch = Branch(history, new_vrptw, root_routes, branch_priority, root_obj)

    solve_BnB!(best_sol, initial_branch)        
end

function complete_BnB!(best_sol, vrptw, root_y, root_routes, root_obj)
    # Most Priority is at the end.
    branch_priority = generate_branch_priority(root_y, root_routes)
    history = OrderedDict()

    root_branch = Branch(history, vrptw, root_routes, branch_priority, root_obj)

    solve_BnB!(best_sol, root_branch)
end

function solve_vrp_bnb(vrptw::VRPTW_Instance; tw_reduce=true)

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
    root_y, root_routes, root_obj = solve_cg_rmp(vrptw, initial_routes=[], tw_reduce=false)
    println("Root node solved. Cumulative Time: ", time() - start_time)
    @show length(root_routes)

    @info("Root LP relaxation solution:")
    @show root_obj
    show_current(root_y, root_routes)
    println("Is the root solution binary? ", is_binary(root_y))

    if is_binary(root_y)
        best_sol.y = root_y 
        best_sol.routes = root_routes 
        best_sol.objective = root_obj
    else            

        initial_BnB!(best_sol, vrptw, root_y, root_routes, root_obj)

        @info("Initial BnB is done.")
        println("Initial BnB. Cumulative Time: ", time() - start_time)
        @show best_sol.objective
        show_current(best_sol.y, best_sol.routes)

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
