function initialize_label(origin, n_nodes; cost=0)
    flag = zeros(Int, n_nodes)
    flag[origin] = 1
    return Label(0.0, 0.0, flag, cost, [origin])
end

function save_forward_star(pg::ESPPRC_Instance; sorted=true)
    n_nodes = length(pg.service_time)
    return save_forward_star(n_nodes, pg.cost; sorted=sorted)
end
function save_forward_star(n_nodes, cost; sorted=true)
    fs = Vector{Vector{Int}}(undef, n_nodes)
    for v_i in 1:n_nodes
        forward_star = findall(x -> x < Inf, cost[v_i, :])
        if sorted 
            sort!(forward_star, by= x->cost[v_i, x])
        end
        fs[v_i] = forward_star 
    end
    return fs 
end
function save_forward_star!(pg::ESPPRC_Instance; sorted=true)
    n_nodes = length(pg.service_time)
    for v_i in 1:n_nodes
        forward_star = findall(x -> x < Inf, pg.cost[v_i, :])
        if sorted 
            sort!(forward_star, by= x->pg.cost[v_i, x])
        end
        pg.forward_star[v_i] = forward_star 
    end
end

function save_reverse_star(pg::ESPPRC_Instance; sorted=true)
    n_nodes = length(pg.service_time)
    return save_reverse_star(n_nodes, pg.cost; sorted=sorted)
end
function save_reverse_star(n_nodes, cost; sorted=true)
    rs = Vector{Vector{Int}}(undef, n_nodes)
    for v_i in 1:n_nodes
        reverse_star = findall(x -> x < Inf, cost[:, v_i])
        if sorted 
            sort!(reverse_star, by= x->cost[x, v_i])
        end
        rs[v_i] = reverse_star 
    end
    return rs
end
function save_reverse_star!(pg::ESPPRC_Instance; sorted=true)
    n_nodes = length(pg.service_time)
    for v_i in 1:n_nodes
        reverse_star = findall(x -> x < Inf, pg.cost[:, v_i])
        if sorted 
            sort!(reverse_star, by= x->pg.cost[x, v_i])
        end
        pg.reverse_star[v_i] = reverse_star 
    end
end

function graph_reduction!(pg::ESPPRC_Instance)
    n_nodes = length(pg.early_time)
    OD = [pg.origin, pg.destination]

    pg.cost[pg.origin, pg.destination] = Inf

    for j in 1:n_nodes
        pg.cost[j, j] = Inf
        for i in 1:n_nodes
            if i != j && !in(i, OD) && !in(j, OD)
                if pg.early_time[i] + pg.service_time[i] + pg.time[i, j] > pg.late_time[j]
                    pg.cost[i, j] = Inf
                end
            end
        end
    end

    # Inclusion of updating fs leads to increased iterations. Don't know why.
    # save_forward_star!(pg)
    # save_reverse_star!(pg)
end


function forward_reach(λ_i::Label, v_i::Int, v_j::Int, pg::ESPPRC_Instance)
    # Check time 
    arrival_time = max( λ_i.time + pg.service_time[v_i] + pg.time[v_i, v_j] , pg.early_time[v_j] )
    if arrival_time > pg.late_time[v_j] - EPS
        return false, nothing, nothing
    end

    # Check capacity
    new_load = λ_i.load + pg.load[v_i, v_j]
    if new_load > pg.capacity - EPS
        return false, nothing, nothing
    end

    return true, arrival_time, new_load
end

function backward_reach(λ_i::Label, v_i::Int, v_k::Int, pg::ESPPRC_Instance)
    # Currently at v_i 
    # extending arc: (v_k, v_i)

    a_bw_k = pg.early_time[v_k] + pg.service_time[v_k]
    b_bw_k = pg.late_time[v_k] .+ pg.service_time[v_k]

    # Check time 
    max_T = pg.info["max_T"]
    min_time_required = max(pg.time[v_k, v_i] + pg.service_time[v_i] + λ_i.time, max_T - b_bw_k)
    if min_time_required > max_T - a_bw_k - EPS
        return false, nothing, nothing
    end

    # Check capacity
    new_load = λ_i.load + pg.load[v_k, v_i]
    if new_load > pg.capacity - EPS
        return false, nothing, nothing
    end

    # @show is_reachable, v_k, λ_i.path, min_time_required
    return true, min_time_required, new_load
end

function calculate_max_T(pg::ESPPRC_Instance)
    n_nodes = length(pg.early_time)
    set_N = 1:n_nodes
    tmp = [
        pg.late_time[i] + pg.service_time[i] + pg.time[i, pg.destination] 
        for i in set_N if pg.time[i, pg.destination] < Inf
    ]
    max_T = min(maximum(tmp), pg.late_time[pg.destination])
    return max_T
end


function calculate_path_time(path::Vector{Int}, pg::ESPPRC_Instance)
    total_time = 0
    for k in 1:length(path)-1
        i, j = path[k], path[k+1]
        total_time = max(total_time + pg.service_time[i] + pg.time[i, j], pg.early_time[j])
    end
    return total_time
end


function calculate_path_cost(new_route::Vector{Int}, cost_mtx::Matrix{Float64})
    r_cost = 0.0
    for i in 1:length(new_route)-1
        r_cost += cost_mtx[new_route[i], new_route[i+1]]
    end
    return r_cost
end


function find_best_label!(labels::Vector{Label})
    if isempty(labels)
        return Label(0, 0, [], Inf, [])
    else
        sort!(labels, by=x->x.cost)
        best_label = labels[1]
        return best_label
    end
end


function is_binary(y::Vector{Float64})
    tol = 1e-6
    for i in eachindex(y)
        if ! ( isapprox(y[i], 1.0, atol=tol) || isapprox(y[i], 0.0, atol=tol) )
            return false
        end
    end
    return true
end
