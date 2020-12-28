# Leonardo Lozano, Daniel Duque, Andrés L. Medaglia (2016) An Exact Algorithm for the Elementary Shortest Path Problem with Resource Constraints. Transportation Science 50(1):348-357. https://doi.org/10.1287/trsc.2014.0582

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

function bounding_time_index(current_time::Float64, btimes::Vector{Float64})
    # time_values (sorted from greatest to smallest)
    Δ = btimes[1] - btimes[2] 
    time_ub = btimes[1]
    k = Int(ceil((time_ub - current_time) / Δ)) + 1
    if k <= length(btimes) 
        @assert current_time >= btimes[k] - EPS
    end
    return k
end


function isbounded(p::Label, primal_bounds::Vector{Label}, lower_bounds::Matrix{Float64}, btimes::Vector{Float64}, pg::ESPPRC_Instance, bounding::Bool; root=pg.origin)
    if isempty(primal_bounds) || isempty(primal_bounds)
        @warn("isempty primal_bounds or primal_bounds")
        return false
    end
    
    v_i = p.path[end]
    bound = bounding ? min_lower_bound = minimum(lower_bounds[root, :]) : primal_bounds[root].cost

    # must use from the bound matrix B the lower closest value to τ available. 
    # time_values (sorted from greatest to smallest)
    # τ <= p.time
    k = bounding_time_index(p.time, btimes)
    if k > length(btimes)
        # In this case, p.time < btimes[end]; no lower_bounds info available.
        return false
    elseif lower_bounds[v_i, k] < Inf
        @assert btimes[k] <= p.time + EPS
        # The condition below should be strict inequality. 
        # If it is set >=, then some problems cannot be solved optimally.
        bounded = p.cost + lower_bounds[v_i, k] > bound + EPS
        return bounded
    else
        return false 
    end
end



function bounding_scheme!(btimes::Vector{Float64}, neg_cost_routes::Vector{Label}, pg::ESPPRC_Instance)
    n_nodes = length(pg.service_time)
    set_N = 1:n_nodes 

    primal_bounds = initialize_label.(set_N, n_nodes; cost=Inf)
    # best_labels = Array{Label, 1}(undef, n_nodes)
    # for i in eachindex(best_labels)
    #     best_labels[i] = initialize_label(i, n_nodes; cost=Inf)
    # end

    # lower_bounds = Matrix{Label}(undef, n_nodes, length(btimes))
    # for i in 1:n_nodes, k in 1:length(btimes)
    #     lower_bounds[i, k] = initialize_label(i, n_nodes; cost=Inf)
    # end    
    lower_bounds = fill(Inf, n_nodes, length(btimes))

    bounding_iteration = 0
    for k in 1:length(btimes)
        cur_time = btimes[k]

        for v_i in 1:n_nodes
            if v_i != pg.destination
                p = initialize_label(v_i, n_nodes)
                if cur_time <= pg.late_time[v_i]
                    p.time = max(cur_time, pg.early_time[v_i])
                    pulse_procedure!(p, primal_bounds, lower_bounds, btimes, neg_cost_routes, pg; 
                                        init_time=cur_time, bounding=true, root=v_i)
                end
                bounding_iteration += 1
            end
        end
    end

    # @show btimes
    # for i in 1:n_nodes, k in 1:length(btimes)
    #     @show k, i, btimes[k], lower_bounds[i, k].cost
    # end    

    # @show bounding_iteration

    return primal_bounds, lower_bounds
end # bounding_scheme


function pulse_procedure!(p::Label, primal_bounds::Vector{Label}, lower_bounds::Matrix{Float64}, btimes::Vector{Float64}, neg_cost_routes::Vector{Label}, pg::ESPPRC_Instance; init_time=0.0, bounding=false, root=pg.origin)
    if length(neg_cost_routes) >= pg.info["max_neg_cost_routes"]
        return
    end

    # current node
    v_i = p.path[end]
    if p.time > pg.late_time[v_i]
        return
    end
    p.time = max(p.time, pg.early_time[v_i])
    global counter += 1

    if v_i == pg.destination   # Arrived at the destination. Update the best bounds
        if p.cost < primal_bounds[root].cost 
            primal_bounds[root] = deepcopy(p)
        end
        if bounding
            k = bounding_time_index(init_time, btimes)
            if k <= length(btimes)
                if p.cost < lower_bounds[root, k] - EPS
                    for kk in k:length(btimes)
                        # lower_bounds[root, kk] = deepcopy(p)
                        lower_bounds[root, kk] = p.cost
                    end            
                end
            end
        end
        if root == pg.origin && p.cost < 0
            push!(neg_cost_routes, p)
        end
        return    
    end

    # If it didn't arrive at the destination yet 
    if isbounded(p, primal_bounds, lower_bounds, btimes, pg, bounding, root=root) 
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
                pulse_procedure!(pp, primal_bounds, lower_bounds, btimes, neg_cost_routes, pg; 
                            init_time=init_time, bounding=bounding, root=root)
            end
        end
    end
    
    return 
end

function solveESPPRCpulse(org_pg::ESPPRC_Instance; step=-1, max_neg_cost_routes=MAX_INT::Int)
    pg = deepcopy(org_pg)
    graph_reduction!(pg)
    pg.info["max_neg_cost_routes"] = max_neg_cost_routes

    n_nodes = length(pg.late_time)

    # Discrete Time Steps for Bounding
    time_ub = calculate_max_T(pg)
    time_lb = 0.1 * time_ub
    if step == -1 
        step = Int(floor(time_ub*0.9 / 10)) + 1    
    end
    btimes = collect(time_ub-step : -step : time_lb)

    # Container for negative reduced cost solutions
    neg_cost_routes = Vector{Label}(undef,0)

    # Bounding Scheme 
    primal_bounds, lower_bounds = bounding_scheme!(btimes, neg_cost_routes, pg)

    p = initialize_label(pg.origin, n_nodes)
    pulse_procedure!(p, primal_bounds, lower_bounds, btimes, neg_cost_routes, pg)

    if max_neg_cost_routes < MAX_INT
        return find_best_label!(neg_cost_routes), neg_cost_routes
    else
        return primal_bounds[pg.origin]
    end
end