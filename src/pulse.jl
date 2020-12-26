# Leonardo Lozano, Daniel Duque, Andrés L. Medaglia (2016) An Exact Algorithm for the Elementary Shortest Path Problem with Resource Constraints. Transportation Science 50(1):348-357. https://doi.org/10.1287/trsc.2014.0582
# Implemented by Changhyun Kwon chkwon@gmail.com


# In the Pulse data structure,
# cost, load, time have been updated for [path; next].
# That is, path is updated by [path; next] later,
# only after all feasbility & pruning checks.

# mutable struct Label
#     time        ::Float64
#     load        ::Float64
#     flag        ::Vector{Int}
#     cost        ::Float64
#     path        ::Vector{Int}
# end



function should_rollback(p::Label, pg::ESPPRC_Instance)
    # Section 4.3 Rollback Pruning
    if length(p.path) < 3
        return false
    end

    v_i = p.path[end-2]
    v_k = p.path[end-1]
    v_j = p.path[end]

    # p: ...... v_i -> v_k -> v_j
    # pp: ..... v_i -> v_j 

    cost_p = pg.cost[v_i, v_k] + pg.cost[v_k, v_j] 
    cost_pp = pg.cost[v_i, v_j]

    path_p = p.path
    path_pp = [p.path[1:end-2]; v_j]

    time_p = calculate_path_time(path_p, pg)
    time_pp = calculate_path_time(path_pp, pg)

    if cost_pp <= cost_p && time_pp <= time_p
        # dominated, should rollback
        return true
    else
        # non-dominated, no need to rollback
        return false
    end
end

function time_to_time_value_index(t::Float64, btimes::Vector{Float64})
    # time_values (sorted from greatest to smallest)
    Δ = btimes[1] - btimes[2] 
    time_ub = btimes[1]
    k = Int(ceil((time_ub - t) / Δ)) + 1
    k = max(k, 1)
    return k
end

function isbounded(p::Label, primal_bounds::Vector{Label}, lower_bounds::Matrix{Label}, btimes::Vector{Float64}, pg::ESPPRC_Instance, bounding::Bool)
    if isempty(primal_bounds) || isempty(primal_bounds)
        @warn("isempty primal_bounds or primal_bounds")
        return false
    end
    
    v_i = p.path[end]
    upper_bound = primal_bounds[pg.origin].cost
    min_lower_bound = minimum([lower_bounds[pg.origin, t].cost for t in 1:length(btimes)])
    bound = bounding ? min_lower_bound : upper_bound


    # must use from the bound matrix B the lower closest value to τ available. 
    # time_values (sorted from greatest to smallest)
    # τ <= p.time
    k = time_to_time_value_index(p.time, btimes)
    if k > length(btimes)
        return false
    elseif lower_bounds[v_i, k].cost < Inf
        @assert btimes[k] <= p.time
        # The condition below should be strict inequality. 
        # If it is set >=, then or-tools-example.jl could fail.
        bounded = p.cost + lower_bounds[v_i, k].cost > bound
        return bounded
    else
        return false 
    end
end



function bounding_scheme(btimes::Vector{Float64}, pg::ESPPRC_Instance)
    n_nodes = length(pg.service_time)
    set_N = 1:n_nodes 

    primal_bounds = initialize_label.(set_N, n_nodes; cost=Inf)
    # best_labels = Array{Label, 1}(undef, n_nodes)
    # for i in eachindex(best_labels)
    #     best_labels[i] = initialize_label(i, n_nodes; cost=Inf)
    # end

    lower_bounds = Matrix{Label}(undef, n_nodes, length(btimes))
    for i in 1:n_nodes, k in 1:length(btimes)
        lower_bounds[i, k] = initialize_label(i, n_nodes; cost=Inf)
    end    

    # We will change the origin of _pg 
    
    bound_order = [n_nodes-1; 1:n_nodes-2; n_nodes]

    bounding_iteration = 0
    for k in 1:length(btimes)
        cur_time = btimes[k]

        for v_i in bound_order
            if v_i != pg.destination
                _pg_ = deepcopy(pg) 
                _pg_.origin = v_i

                p = initialize_label(v_i, n_nodes)
                if cur_time <= pg.late_time[v_i]
                    p.time = max(cur_time, _pg_.early_time[v_i])
                    pulse_procedure!(p, primal_bounds, lower_bounds, btimes, _pg_; init_time=cur_time, bounding=true)
                end
                bounding_iteration += 1
            end
        end
    end

    # @show btimes
    # for i in 1:n_nodes, k in 1:length(btimes)
    #     @show k, i, btimes[k], lower_bounds[i, k].cost
    # end    

    @show bounding_iteration

    return primal_bounds, lower_bounds
end # bounding_scheme


function pulse_procedure!(p::Label, primal_bounds::Vector{Label}, lower_bounds::Matrix{Label}, btimes::Vector{Float64}, pg::ESPPRC_Instance; init_time=0.0, bounding=false)
    global counter += 1

    # current node
    v_i = p.path[end]
    if p.time < pg.early_time[v_i]
        p.time = pg.early_time[v_i]
    end    
    if p.time > pg.late_time[v_i]
        return
    end

    if v_i == pg.destination
        if p.cost < primal_bounds[pg.origin].cost 
            primal_bounds[pg.origin] = deepcopy(p)
        end
        k = time_to_time_value_index(init_time, btimes)
        if bounding && k <= length(btimes)
            if p.cost < lower_bounds[pg.origin, k].cost 
                if k == 1 || p.cost <= lower_bounds[pg.origin, k-1].cost 
                    lower_bounds[pg.origin, k] = deepcopy(p)
                end
            end
        end
        return    
        
    else # If it didn't arrive at the destination yet 
        if isbounded(p, primal_bounds, lower_bounds, btimes, pg, bounding) 
            return         
        end
        if should_rollback(p, pg) 
            return
        end

        # Create a new pulse 
        for v_j in pg.forward_star[v_i]
            if p.flag[v_j] == 0 
                isreachable, new_time, new_load = forward_reach(p, v_i, v_j, pg)
                if isreachable 
                    pp = deepcopy(p)
                    push!(pp.path, v_j)
                    pp.cost += pg.cost[v_i, v_j]
                    pp.load = new_load 
                    pp.time = new_time 
                    pp.flag[v_j] = 1 
                    pulse_procedure!(pp, primal_bounds, lower_bounds, btimes, pg; init_time=init_time, bounding=bounding)
                end
            end
        end
                
    end

end


global counter = 0 

function solveESPPRCpulse(org_pg::ESPPRC_Instance; step=10, max_neg_cost_routes=Inf)
    pg = deepcopy(org_pg)
    graph_reduction!(pg)

    n_nodes = length(pg.late_time)

    # Discrete Time Steps for Bounding
    time_ub = calculate_max_T(pg)
    time_lb = 0.1 * time_ub
    step = 200
    btimes = collect(time_ub-step : -step : time_lb)

    # Bounding Scheme 
    primal_bounds, lower_bounds = bounding_scheme(btimes, pg)

    @show counter

    # p = initialize_pulse(pg.origin)
    # best_p = initialize_pulse(pg.origin; cost=Inf)

    # neg_cost_sols = Vector{Pulse}(undef,0)

    p = initialize_label(pg.origin, n_nodes)
    pulse_procedure!(p, primal_bounds, lower_bounds, btimes, pg)
    @show counter

    return primal_bounds[pg.origin]


    # println("total pulse conter:" , counter) 
    # if max_neg_cost_routes < Inf
    #     return convert_to_label(best_p), convert_to_label.(neg_cost_sols)
    # else
    #     return convert_to_label(best_p)
    # end
end