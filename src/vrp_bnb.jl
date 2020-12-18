using DataStructures

include("vrp_cg_rmp.jl")

struct Branch
    branch_mtx::Array
    history::OrderedDict
    vrptw_instance::VRPTW_Instance
    root_routes::Array
    branch_priority::Array
    lower_bound::Float64
end

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

# function remove_arc_in_routes!(routes, i, j)
#     routes_to_be_removed = []
#     for r_idx in eachindex(routes)
#         r = routes[r_idx]
#         i_idx = findfirst(isequal(i), r)
#         if i_idx != nothing 
#             if i_idx != length(r)
#                 if r[i_idx+1] == j 
#                     push!(routes_to_be_removed, r_idx)
#                 end
#             end
#         end
#     end
#     deleteat!(routes, routes_to_be_removed)
# end

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
    new_branch_mtx = copy(current_branch.branch_mtx)
    new_routes = deepcopy(current_branch.root_routes)

    n_nodes = size(new_vrptw.travel_time, 1)

    new_branch_mtx[i, j] = value
    
    update_travel_time!(new_vrptw, arc, value)

    remove_arc_in_routes!(new_routes, new_vrptw.travel_time)

    return new_branch_mtx, new_routes, new_vrptw
end


function select_new_arc(branching_arcs, branch_priority)
    new_branch_priority = copy(branch_priority)

    if isempty(branch_priority)
        idx = findfirst(isequal(-1), branching_arcs)
        arc = (idx[1], idx[2])
    else
        arc = pop!(new_branch_priority)[1]
        while branching_arcs[arc[1], arc[2]] != -1
            arc = pop!(new_branch_priority)[1]
        end
    end

    return arc, new_branch_priority
end

mutable struct BestIncumbent
    y::Vector{Float64}
    routes::Vector{Vector{Int}}
    objective::Float64
end


function show_current(sol_y, sol_routes)
    for n in 1:length(sol_y)
        if sol_y[n] > 0.01
            @show n, sol_y[n], sol_routes[n]
        end
    end    
end


function solve_vrp_bnb(vrptw::VRPTW_Instance; tw_reduce=true)

    function solve_BnB!(best_sol::BestIncumbent, root_branch)
        branches = [root_branch]
        next_branches = Branch[]

        while !isempty(branches)
            branch_counter += 1
            sort!(branches, by= x->x.lower_bound, rev=true)
            current_branch = pop!(branches)
            arc, new_branch_priority = select_new_arc(current_branch.branch_mtx, current_branch.branch_priority)

            sol_y, sol_routes, sol_obj = [], [], Inf
            for arc_value in 0:1

                new_history = copy(current_branch.history)
                new_history[arc] = arc_value

                # println("-------------------------------------------------")
                # @info("New branch: $arc=$arc_value.")
                # @show new_history
            
                new_branch_mtx, new_routes, new_vrptw = generate_branch_instance(arc, arc_value, current_branch)
                
                if isempty(new_routes)
                    # println("new_routes is empty.")
                    sol_y, sol_routes, sol_obj = [], [], Inf
                else
                    sol_y, sol_routes, sol_obj = solve_cg_rmp(new_vrptw, initial_routes=new_routes, tw_reduce=false)
                end

                if isempty(sol_y)
                    # stop. Infeasible
                    # printstyled("*****[bnb]********** Infeasible.\n", color=:red)

                elseif is_binary(sol_y)
                    # printstyled("*****[bnb]********** Binary. $sol_obj (Best=$(best_sol.objective)).\n", color=:yellow)
                    if sol_obj < best_sol.objective
                        # printstyled("*****[bnb]********** New Binary Best $sol_obj.\n", color=:blue)
                        best_sol.y = sol_y
                        best_sol.routes = sol_routes
                        best_sol.objective = sol_obj

                        # show_current(sol_y, sol_routes)
                        # readline()

                    end
                    # stop. a binary feasible solution found. 
                    
                elseif sol_obj < best_sol.objective
                    # Still alive. Move on 
                    # printstyled("*****[bnb]********** Fractional LP Solution Alive. Keep Going $sol_obj (Best=$(best_sol.objective)).\n" , color=:green)
                    push!(next_branches, Branch(new_branch_mtx, new_history, new_vrptw, 
                                                current_branch.root_routes, new_branch_priority, sol_obj))

                else 
                    # stop. Bounded. 
                    # printstyled("*****[bnb]********** Stop. Fathomed. $sol_obj (Best=$(best_sol.objective)).\n" , color=:red)

                end

                # @show length(branches), length(next_branches)
            end

            if isempty(branches)
                branches = deepcopy(next_branches)
                next_branches = Branch[]
            end

            # readline()

            
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

        # Let branch-and-bound begin
        # if -1, then it is not yet branched. 
        # diagonal terms are set to zero.
        branch_mtx = fill(-1, size(new_vrptw.travel_time))
        for i in 1:size(branch_mtx, 1)
            branch_mtx[i, i] = 0
        end
        history = OrderedDict()

        for n in eachindex(root_y)
            if root_y[n] > 1.0 - 1e-6
                r = root_routes[n]
                for i in 1:length(r)-1
                    arc = (r[i], r[i+1])
                    branch_mtx[r[i], r[i+1]] = 1
                    history[arc] = 1
                    update_travel_time!(new_vrptw, arc, 1)
                end
            end
        end                

        initial_branch = Branch(branch_mtx, history, new_vrptw, root_routes, branch_priority, root_obj)

        solve_BnB!(best_sol, initial_branch)        
    end

    function complete_BnB!(best_sol, vrptw, root_y, root_routes, root_obj)
        # Most Priority is at the end.
        branch_priority = generate_branch_priority(root_y, root_routes)

        # Let branch-and-bound begin
        # if -1, then it is not yet branched. 
        # diagonal terms are set to zero.
        branch_mtx = fill(-1, size(vrptw.travel_time))
        for i in 1:size(branch_mtx, 1)
            branch_mtx[i, i] = 0
        end
        history = OrderedDict()

        root_branch = Branch(branch_mtx, history, vrptw, root_routes, branch_priority, root_obj)

        solve_BnB!(best_sol, root_branch)
    end


    # Time Windows Reduction 
    if tw_reduce
        time_window_reduction!(vrptw)
    end

    # initialization
    best_sol = BestIncumbent([], [], Inf)

    # solve root node 
    @info("Solving the root LP relaxation...")
    root_y, root_routes, root_obj = solve_cg_rmp(vrptw, initial_routes=[], tw_reduce=false)
    @show length(root_routes)

    @info("Root LP relaxation solution:")
    @show root_obj
    show_current(root_y, root_routes)
    println("Is the root solution binary? ", is_binary(root_y))

    branch_counter = 0

    # readline()


    if is_binary(root_y)
        best_sol.y = root_y 
        best_sol.routes = root_routes 
        best_sol.objective = root_obj
    else            

        initial_BnB!(best_sol, vrptw, root_y, root_routes, root_obj)

        @info("Initial BnB is done.")
        @show branch_counter
        @show best_sol.objective
        show_current(best_sol.y, best_sol.routes)

        branch_counter = 0
        complete_BnB!(best_sol, vrptw, root_y, root_routes, root_obj)

        @info("Complete BnB is done.")
        @show branch_counter
        @show best_sol.objective
        show_current(best_sol.y, best_sol.routes)

    end

    return best_sol.y, best_sol.routes, best_sol.objective
end
