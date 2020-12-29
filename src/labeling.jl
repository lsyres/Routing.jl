
function show_label(label::Label)
    println("time=$(label.time), load=$(label.load), cost=$(label.cost), path=$(label.path)")
end

function is_identical(label::Label, other_label::Label)
    if label.cost != other_label.cost
        return false
    elseif label.time != other_label.time
        return false
    elseif label.load != other_label.load
        return false
    else
        for i in 1:length(label.flag)
            if label.flag[i] != other_label.flag[i]
                return false
            end
        end
    end
    return true
    # is_same = label.path == other_label.path
    # has_same_values = label.cost == other_label.cost &&
    #                     label.time == other_label.time &&
    #                     label.load == other_label.load &&
    #                     label.flag == other_label.flag

    # return has_same_values
end

function dominate(label::Label, other_label::Label, pg::ESPPRC_Instance)
    # Check if label dominates other_label
    CN = collect(pg.critical_nodes) :: Vector{Int}

    if label.cost > other_label.cost
        return false
    elseif label.time > other_label.time
        return false
    elseif label.load > other_label.load
        return false
    # elseif sum(label.flag[CN]) > sum(label.flag[CN])
    # elseif all(label.flag[CN] .>= other_label.flag[CN])
    # elseif any(label.flag[CN] .> other_label.flag[CN])
        # return false
    else
        for i in CN
            if label.flag[i] > other_label.flag[i]
                return false
            end
        end
    end

    return true

end

function update_flag!(label::Label, pg::ESPPRC_Instance; direction="backward")
    if direction=="forward"
        v_j = label.path[end]
        label.flag[v_j] = 1
        for v_k in successors(v_j, pg)
            if label.flag[v_k] == 0
                is_reachable, _, _ = forward_reach(label, v_j, v_k, pg)
                if !is_reachable
                    label.flag[v_k] = 1
                end
            end
        end
    elseif direction=="backward"
        v_j = label.path[1]
        label.flag[v_j] = 1
        for v_k in predecessors(v_j, pg)
            if label.flag[v_k] == 0
                is_reachable, _, _ = backward_reach(label, v_j, v_k, pg)
                if !is_reachable
                    label.flag[v_k] = 1
                end
            end
        end
    elseif direction=="join"
        # do nothing
    end
end

function EFF!(Λ::Vector{Vector{Label}}, label::Label, v_j::Int, pg::ESPPRC_Instance)
    is_updated = false
    idx = Int[]
    for n in eachindex(Λ[v_j])
        other_label = Λ[v_j][n]
        if dominate(label, other_label, pg) && !is_identical(label, other_label)
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
        if dominate(other_label, label, pg) || is_identical(label, other_label)
            is_non_dominated = false
            break
        end
    end

    if is_non_dominated
        push!(Λ[v_j], label)
        global counter += 1
        is_updated = true
    end

    return is_updated
end


function predecessors(v_i::Int, pg::ESPPRC_Instance)
    if v_i == pg.origin
        return Int[]
    else
        return pg.reverse_star[v_i]
    end
end

function successors(v_i::Int, pg::ESPPRC_Instance)
    if v_i == pg.destination
        return Int[]
    else
        return pg.forward_star[v_i]
    end
end

function forward_extend(λ_i::Label, v_i::Int, v_j::Int, pg::ESPPRC_Instance)
    is_reachable, new_time, new_load = forward_reach(λ_i, v_i, v_j, pg)

    if !is_reachable
        λ_i.flag[v_j] = 1
        return nothing
    end
    new_cost = λ_i.cost + pg.cost[v_i, v_j]
    new_path = copy(λ_i.path)
    push!(new_path, v_j)

    label = Label(new_time, new_load, copy(λ_i.flag), new_cost, new_path)
    update_flag!(label, pg, direction="forward")

    return label
end

function backward_extend(λ_i::Label, v_i::Int, v_k::Int, pg::ESPPRC_Instance)
    # Currently at v_i
    # extending arc: (v_k, v_i)

    is_reachable, new_time, new_load = backward_reach(λ_i, v_i, v_k, pg)

    if !is_reachable
        λ_i.flag[v_k] = 1
        return nothing
    end
    new_cost = pg.cost[v_k, v_i] + λ_i.cost
    new_path = copy(λ_i.path)
    pushfirst!(new_path, v_k)

    label = Label(new_time, new_load, copy(λ_i.flag), new_cost, new_path)
    update_flag!(label, pg, direction="backward")

    return label
end


function prepare_return_values!(labels::Vector{Label}, max_neg_routes::Int)
    if isempty(labels)
        return Label(0, 0, [], Inf, []), []
    else
        best_label = find_best_label!(labels)
        idx = findfirst(x -> x.cost >= 0.0, labels)
        if  idx == 1
            neg_cost_routes = []
        elseif isnothing(idx)
            neg_cost_routes = labels
        else
            idx = min(idx-1, max_neg_routes)
            neg_cost_routes = labels[1:idx]
        end
        return best_label, neg_cost_routes
    end
end

function join_labels!(final_labels::Vector{Label}, λ_i::Label, λ_j::Label, pg::ESPPRC_Instance)
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
    max_T = pg.max_T
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

function select_node!(set_E::Vector{Int}, pg::ESPPRC_Instance)
    if isempty(set_E)
        @error("The candidate set is empty.")
    else
        return set_E[1]
    end
end

function forward_search!(v_i::Int, Λ_fw::Vector{Vector{Label}}, set_E::Vector{Int}, neg_cost_routes::Vector{Label}, pg::ESPPRC_Instance)
    max_T = pg.max_T :: Float64
    # Forward Extension
    for λ_i in Λ_fw[v_i]
        if λ_i.time <= (1/2) * max_T
            for v_j in successors(v_i, pg)
                if λ_i.flag[v_j] == 0 || !in(v_j, pg.critical_nodes)
                    label = forward_extend(λ_i, v_i, v_j, pg)
                    if !isnothing(label)
                        is_updated = EFF!(Λ_fw, label, v_j, pg)
                        if is_updated
                            global counter1 += 1
                            push!(set_E, v_j)
                        end
                        if v_j == pg.destination && label.cost < 0.0
                            push!(neg_cost_routes, label)
                        end
                    end
                end
            end
        end
    end
end

function backward_search!(v_i::Int, Λ_bw::Vector{Vector{Label}}, set_E::Vector{Int}, pg::ESPPRC_Instance)
    max_T = pg.max_T :: Float64
    for λ_i in Λ_bw[v_i]
        if λ_i.time <= (1/2) * max_T
            for v_k in predecessors(v_i, pg)
                if λ_i.flag[v_k] == 0 || !in(v_k, pg.critical_nodes)
                    label = backward_extend(λ_i, v_i, v_k, pg)
                    if !isnothing(label)
                        is_updated = EFF!(Λ_bw, label, v_k, pg)
                        if is_updated
                            global counter2 += 1
                            push!(set_E, v_k)
                        end
                    end
                end
            end
        end
    end
end


function join_label_sets(Λ_fw::Vector{Vector{Label}}, Λ_bw::Vector{Vector{Label}}, pg::ESPPRC_Instance)
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
                                    new_cost = join_labels!(final_labels,λ_i, λ_j, pg)
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


function find_cycle_nodes(path::Vector{Int}, n_nodes::Int)
    visit_count = zeros(Int, n_nodes)
    for i in path
        visit_count[i] += 1
    end
    cycle_nodes = findall(x->x>1, visit_count)
    return cycle_nodes
end

function monodirectional(org_pg::ESPPRC_Instance; max_neg_routes=MAX_INT::Int, DSSR=false)
    # Feillet, D., Dejax, P., Gendreau, M., Gueguen, C., 2004. An exact algorithm for the elementary shortest path problem with resource constraints: Application to some vehicle routing problems. Networks 44, 216–229. https://doi.org/10.1002/net.20033
    pg = deepcopy(org_pg)
    graph_reduction!(pg)
    pg.max_neg_routes = max_neg_routes

    n_nodes = length(pg.early_time)
    set_N = 1:n_nodes
    pg.max_T = Inf
    if DSSR
        pg.critical_nodes = Set(Int[])
    else
        pg.critical_nodes = Set(set_N)
    end

    neg_cost_routes = Label[]

    final_labels = Label[]
    best_label = nothing
    has_cycle = true
    CN_iter = 1
    global counter = 0


    # Searchinc
    while has_cycle
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

        while !isempty(set_E)
            # v_i = set_E[1]
            v_i = select_node!(set_E, pg) :: Int
            forward_search!(v_i, Λ_fw, set_E, neg_cost_routes, pg)

            if length(neg_cost_routes) >= max_neg_routes
                break
            end
            setdiff!(set_E, v_i)
        end

        best_label = find_best_label!(Λ_fw[pg.destination])
        cycle_nodes = find_cycle_nodes(best_label.path, n_nodes)
        if isempty(cycle_nodes)
            has_cycle = false
            final_labels = Λ_fw[pg.destination]

            @show counter
            @show length(final_labels)
            total_labels = 0
            for L in Λ_fw
                total_labels += length(L)
            end
            @show total_labels
        else
            @show cycle_nodes
            union!(pg.critical_nodes, cycle_nodes)
            CN_iter += 1
        end
    end
    # @show size(final_labels)

    # @show CN_iter



    return prepare_return_values!(final_labels, max_neg_routes)

end

function bidirectional(org_pg::ESPPRC_Instance; max_neg_routes=MAX_INT::Int, DSSR=false)
    # Righini, G., Salani, M., 2006. Symmetry helps: Bounded bi-directional dynamic programming for the elementary shortest path problem with resource constraints. Discrete Optimization 3, 255–273. https://doi.org/10.1016/j.disopt.2006.05.007

    pg = deepcopy(org_pg)
    graph_reduction!(pg)

    n_nodes = length(pg.early_time)
    set_N = 1:n_nodes
    pg.max_T = calculate_max_T(pg)
    if DSSR
        pg.critical_nodes = Set(Int[])
    else
        pg.critical_nodes = Set(collect(set_N))
    end

    final_labels = Label[]
    best_label = nothing
    has_cycle = true
    CN_iter = 0
    global counter = 0
    global counter1 = 0
    global counter2 = 0

    # Search
    while has_cycle
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
            tmp = time()
            forward_search!(v_i, Λ_fw, set_E, Label[], pg)
            forward_cpu_time += time() - tmp

            tmp = time()
            backward_search!(v_i, Λ_bw, set_E, pg)
            backward_cpu_time += time() - tmp

            setdiff!(set_E, v_i)
        end

        join_cpu_time = time()
        final_labels, count_fw_labels, count_bw_labels = join_label_sets(Λ_fw, Λ_bw, pg)
        join_cpu_time = time() - join_cpu_time
        # @show deci3.([forward_cpu_time, backward_cpu_time, join_cpu_time])

        best_label = find_best_label!(final_labels)
        cycle_nodes = find_cycle_nodes(best_label.path, n_nodes)
        if isempty(cycle_nodes)
            has_cycle = false

            @show counter, counter1, counter2
            @show length(final_labels)
            total_labels = 0
            for L in Λ_fw
                total_labels += length(L)
            end
            for L in Λ_bw
                total_labels += length(L)
            end
            @show total_labels

        else
            union!(pg.critical_nodes, cycle_nodes)
            CN_iter += 1
        end
    end

    # @show CN_iter

    # @show count_fw_labels, count_bw_labels, size(final_labels)
    return prepare_return_values!(final_labels, max_neg_routes)

end
