
function predecessors(v_i, pg::ESPPRC_Instance)
    if v_i == pg.origin
        return []
    else
        pred = setdiff(findall(x -> x < Inf, pg.cost[:, v_i]), [v_i])
        # sort!(pred, by=x->pg.time[x, v_i])
        return pred
    end
end

function successors(v_i, pg::ESPPRC_Instance)
    if v_i == pg.destination
        return []
    else
        succ = setdiff(findall(x -> x < Inf, pg.cost[v_i, :]), [v_i])
        # sort!(succ, by=x->pg.time[v_i, x])
        return succ
    end
end

function forward_reach(λ_i::Label, v_i::Int, v_j::Int, pg::ESPPRC_Instance)
    is_reachable = true

    # Check time 
    arrival_time = max( λ_i.time + pg.service_time[v_i] + pg.time[v_i, v_j] , pg.early_time[v_j] )
    if arrival_time > pg.late_time[v_j]
        is_reachable = false
    end

    # Check capacity
    new_load = λ_i.load + pg.load[v_i, v_j]
    if new_load > pg.capacity
        is_reachable = false
    end

    return is_reachable, arrival_time, new_load
end



function backward_reach(λ_i::Label, v_i::Int, v_k::Int, pg::ESPPRC_Instance, max_T)
    # Currently at v_i 
    # extending arc: (v_k, v_i)

    is_reachable = true

    a_bw_k = pg.early_time[v_k] + pg.service_time[v_k]
    b_bw_k = pg.late_time[v_k] .+ pg.service_time[v_k]

    # Check time 
    min_time_required = max(pg.time[v_k, v_i] + pg.service_time[v_i] + λ_i.time, max_T - b_bw_k)
    if min_time_required > max_T - a_bw_k
        is_reachable = false
    end

    # Check capacity
    new_load = λ_i.load + pg.load[v_k, v_i]
    if new_load > pg.capacity
        is_reachable = false
    end

    # @show is_reachable, v_k, λ_i.path, min_time_required
    return is_reachable, min_time_required, new_load
end

function update_flag!(label::Label, pg::ESPPRC_Instance; direction="backward")    
    if direction=="forward"
        v_j = label.path[end]
        label.flag[v_j] = 1
        # for v_k in successors(v_j, pg)
        #     if label.flag[v_k] == 0
        #         is_reachable, _, _ = forward_reach(label, v_j, v_k, pg)
        #         if !is_reachable
        #             label.flag[v_k] = 1
        #         end
        #     end
        # end
    elseif direction=="backward"
        v_j = label.path[1]
        label.flag[v_j] = 1
    
    elseif direction=="join"
    
    end
end

function forward_extend(λ_i::Label, v_i::Int, v_j::Int, pg::ESPPRC_Instance)
    is_reachable, new_time, new_load = forward_reach(λ_i, v_i, v_j, pg)

    if !is_reachable
        # λ_i.flag[v_j] = 1
        return nothing
    end
    new_cost = λ_i.cost + pg.cost[v_i, v_j]
    new_path = [λ_i.path; v_j]

    label = Label(new_time, new_load, copy(λ_i.flag), new_cost, new_path)
    update_flag!(label, pg, direction="forward")

    return label
end

function backward_extend(λ_i::Label, v_i::Int, v_k::Int, pg::ESPPRC_Instance, max_T)
    # Currently at v_i 
    # extending arc: (v_k, v_i)

    is_reachable, new_time, new_load = backward_reach(λ_i, v_i, v_k, pg, max_T)

    if !is_reachable
        # λ_i.flag[v_j] = 1
        return nothing
    end
    new_cost = pg.cost[v_k, v_i] + λ_i.cost 
    new_path = [v_k; λ_i.path]

    label = Label(new_time, new_load, copy(λ_i.flag), new_cost, new_path)
    update_flag!(label, pg, direction="backward")

    return label
end


function show_label(label::Label)
    println("time=$(label.time), load=$(label.load), cost=$(label.cost), path=$(label.path)")
end

function is_identical(label::Label, other_label::Label)
    is_same = label.path == other_label.path
    has_same_values = label.cost == other_label.cost &&
                        label.time == other_label.time && 
                        label.load == other_label.load

    if is_same
        # Exactly the same path
        if !has_same_values
            @warn("same path, but different values")
            show_label(label)
            show_label(other_label)
            @show dominate(label, other_label)
            @show dominate(other_label, label)
            @show [label.flag'; other_label.flag']
        end
        return true
    else 
        return false
    end
end
                    
function dominate(label::Label, other_label::Label)
    # Check if label dominates other_label 

    if label.cost > other_label.cost
        return false
    elseif label.time > other_label.time
        return false
    elseif label.load > other_label.load 
        return false
    elseif all(label.flag .>= other_label.flag)
        return false
    else
        return true
    end
end


function EFF!(Λ::Vector, label::Label, v_j::Int)
    is_updated = false
    idx = []
    for n in eachindex(Λ[v_j])
        other_label = Λ[v_j][n]
        if dominate(label, other_label) && !is_identical(label, other_label)
            push!(idx, n)
        end
    end

    if !isempty(idx)
        deleteat!(Λ[v_j], idx)
        is_updated = true
    end

    is_non_dominated = true 
    for n in eachindex(Λ[v_j])
        other_label = Λ[v_j][n]
        if dominate(other_label, label) || is_identical(label, other_label)
            is_non_dominated = false
            break
        end
    end

    if is_non_dominated
        push!(Λ[v_j], label)
        is_updated = true
    end

    return is_updated
end

function prepare_return_values!(labels, max_neg_cost_routes)
    if isempty(labels)
        if max_neg_cost_routes < Inf
            return Label(0, 0, [], Inf, []), []
        else
            return Label(0, 0, [], Inf, []) 
        end
    else
        sort!(labels, by=x->x.cost)
        best_label = labels[1]
        if max_neg_cost_routes < Inf
            idx = findfirst(x -> x.cost >= 0.0, labels)
            if  idx == 1
                neg_cost_labels = []
            elseif isnothing(idx)
                neg_cost_labels = labels
            else
                idx = min(idx-1, max_neg_cost_routes)
                neg_cost_labels = labels[1:idx]
            end
            return best_label, neg_cost_labels
        else
            return best_label
        end
    end
end

function join_labels!(final_labels, λ_i::Label, λ_j::Label, pg::ESPPRC_Instance, max_T)
    v_i = λ_i.path[end] # forward label
    v_j = λ_j.path[1]   # backward label

    # Check no cycle
    new_flag = λ_i.flag .+ λ_j.flag
    if !prod(λ_i.flag .+ λ_j.flag .<= 1)
        return Inf
    end
    # Check capacity
    new_load = λ_i.load + pg.load[v_i, v_j] + λ_j.load 
    if new_load > pg.capacity
        return Inf
    end
    # Check time
    time_check = λ_i.time + pg.service_time[v_i] + pg.time[v_i, v_j] + pg.service_time[v_j] + λ_j.time
    if time_check > max_T
        return Inf
    end

    new_path = [λ_i.path; λ_j.path]
    new_cost = λ_i.cost + pg.cost[v_i, v_j] + λ_j.cost 
    new_time = calculate_path_time(new_path, pg)
    
    new_label = Label(new_time, new_load, new_flag, new_cost, new_path)
    update_flag!(new_label, pg, direction="join")
    push!(final_labels, new_label)

    return new_cost
end

function select_node!(set_E, pg::ESPPRC_Instance)
    return set_E[1]
end


function forward_search!(v_i::Int, Λ_fw::Array, set_E::Array, neg_cost_labels::Vector{Label}, pg::ESPPRC_Instance; max_time=Inf)
    # Forward Extension            
    for λ_i in Λ_fw[v_i]
        if λ_i.time <= max_time
            for v_j in successors(v_i, pg)
                if λ_i.flag[v_j] == 0 && !in(v_j, λ_i.path)
                    label = forward_extend(λ_i, v_i, v_j, pg)
                    if !isnothing(label)
                        is_updated = EFF!(Λ_fw, label, v_j)
                        if is_updated
                            push!(set_E, v_j)
                        end    
                        if v_j == pg.destination && label.cost < 0.0
                            push!(neg_cost_labels, label)
                        end                                                
                    end
                end
            end
        end
    end
end

function backward_search!(v_i::Int, Λ_bw::Array, set_E::Array, pg::ESPPRC_Instance, max_T::Float64; max_time=Inf)
    for λ_i in Λ_bw[v_i]
        if λ_i.time < max_time
            for v_k in predecessors(v_i, pg)
                if λ_i.flag[v_k] == 0 && !in(v_k, λ_i.path)
                    label = backward_extend(λ_i, v_i, v_k, pg, max_T)
                    if !isnothing(label)
                        is_updated = EFF!(Λ_bw, label, v_k)
                        if is_updated
                            push!(set_E, v_k)
                        end    
                    end
                end
            end
        end
    end
end


function join_label_sets(Λ_fw, Λ_bw, max_T, pg::ESPPRC_Instance)
    n_nodes = length(pg.early_time)
    set_N = 1:n_nodes

    # Finding the minimum cost among labels
    min_all_bw = Inf 
    min_bw = fill(Inf, n_nodes)
    min_fw = fill(Inf, n_nodes)
    count_fw_labels = 0
    count_bw_labels = 0
    for v_i in set_N
        for λ_i in Λ_fw[v_i]
            min_fw[v_i] = min(min_fw[v_i], λ_i.cost)
            count_fw_labels += 1
        end
        for λ_i in Λ_bw[v_i]
            min_bw[v_i] = min(min_bw[v_i], λ_i.cost)
            min_all_bw = min(min_all_bw, λ_i.cost)
            count_bw_labels += 1
        end
    end
    UB = Inf

    final_labels = Label[]
    # Join between forward and backward paths
    for v_i in set_N
        min_c = minimum([pg.cost[v_i,j] for j in set_N if j!=v_i])
        if min_fw[v_i] + min_c + min_all_bw < UB
            for λ_i in Λ_fw[v_i]
                if λ_i.cost + min_c + min_all_bw < UB 
                    for v_j in set_N
                        if λ_i.cost + pg.cost[v_i,v_j] + min_bw[v_j] < UB 
                            for λ_j in Λ_bw[v_j]
                                if λ_i.cost + pg.cost[v_i,v_j] + λ_j.cost < UB 
                                    new_cost = join_labels!(final_labels,λ_i, λ_j, pg, max_T)
                                    UB = min(UB, new_cost)
                                end
                            end
                        end
                    end
                end
            end
        end
    end
    return final_labels, count_fw_labels, count_bw_labels
end



function monodirectional(org_pg::ESPPRC_Instance; max_neg_cost_routes=Inf)
    # Feillet, D., Dejax, P., Gendreau, M., Gueguen, C., 2004. An exact algorithm for the elementary shortest path problem with resource constraints: Application to some vehicle routing problems. Networks 44, 216–229. https://doi.org/10.1002/net.20033

    pg = deepcopy(org_pg)
    graph_reduction!(pg)

    n_nodes = length(pg.early_time)
    set_N = 1:n_nodes

    neg_cost_labels = Label[]

    # Initial Label Set
    Λ_fw = Vector{Vector{Label}}(undef, n_nodes)
    for v_i in set_N
        Λ_fw[v_i] = Label[]
    end

    # Label at the origin
    unreachable = zeros(Int, n_nodes)
    init_label = Label(pg.early_time[pg.origin], 0.0, unreachable, 0.0, [pg.origin])
    update_flag!(init_label, pg, direction="forward")
    push!(Λ_fw[pg.origin], init_label)

    # Inititial search nodes
    set_E = [pg.origin]

    forward_cpu_time = 0

    # Search
    while !isempty(set_E)
        # v_i = set_E[1]
        v_i = select_node!(set_E, pg)
        forward_search!(v_i, Λ_fw, set_E, neg_cost_labels, pg)

        if length(neg_cost_labels) >= max_neg_cost_routes
            break
        end
        setdiff!(set_E, v_i)
    end

    final_labels = Λ_fw[pg.destination]
    # @show size(final_labels)
    
    return prepare_return_values!(final_labels, max_neg_cost_routes)

end

function bidirectional(org_pg::ESPPRC_Instance; max_neg_cost_routes=Inf)
    # Righini, G., Salani, M., 2006. Symmetry helps: Bounded bi-directional dynamic programming for the elementary shortest path problem with resource constraints. Discrete Optimization 3, 255–273. https://doi.org/10.1016/j.disopt.2006.05.007

    pg = deepcopy(org_pg)
    graph_reduction!(pg)

    n_nodes = length(pg.early_time)
    set_N = 1:n_nodes
    max_T = calculate_max_T(pg)

    # Initial Label Sets
    Λ_fw = Vector{Vector{Label}}(undef, n_nodes)
    Λ_bw = Vector{Vector{Label}}(undef, n_nodes)
    for v_i in set_N
        Λ_fw[v_i] = Label[]
        Λ_bw[v_i] = Label[]
    end

    # Label at the origin
    unreachable = zeros(Int, n_nodes)
    init_label = Label(pg.early_time[pg.origin], 0.0, unreachable, 0.0, [pg.origin])
    update_flag!(init_label, pg, direction="forward")
    push!(Λ_fw[pg.origin], init_label)

    # Label at the destination
    unreachable = zeros(Int, n_nodes)
    term_label = Label(0.0, 0.0, unreachable, 0.0, [pg.destination])
    update_flag!(term_label, pg, direction="backward")
    push!(Λ_bw[pg.destination], term_label)

    # Inititial search nodes
    set_E = [pg.origin, pg.destination]

    forward_cpu_time = 0
    backward_cpu_time = 0
    join_cpu_time = 0

    # Search
    while !isempty(set_E)
        # v_i = set_E[1]
        v_i = select_node!(set_E, pg)
        forward_search!(v_i, Λ_fw, set_E, Label[], pg; max_time=max_T/2)
        backward_search!(v_i, Λ_bw, set_E, pg, max_T; max_time=max_T/2)
        setdiff!(set_E, v_i)
    end

    final_labels, count_fw_labels, count_bw_labels = join_label_sets(Λ_fw, Λ_bw, max_T, pg)

    # @show count_fw_labels, count_bw_labels, size(final_labels)
    return prepare_return_values!(final_labels, max_neg_cost_routes)

end