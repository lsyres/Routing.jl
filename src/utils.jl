function initialize_label(origin, n_nodes; cost=0)
    flag = zeros(Int, n_nodes)
    flag[origin] = 1
    return Label(0.0, 0.0, flag, cost, [origin])
end

function save_forward_star(pg::ESPPRC_Instance; sorted=true)
    n_nodes = length(pg.service_time)
    return save_forward_star(n_nodes, pg.cost; sorted=sorted)
end
function save_forward_star(n_nodes::Int, destination::Int, cost::Matrix{Float64}; sorted=true)
    forward_star = Vector{Vector{Int}}(undef, n_nodes)
    for v_i in 1:n_nodes
        if v_i == destination
            forward_star[v_i] = Int[]
        else
            fs = findall(x -> x < Inf, cost[v_i, :])
            if sorted 
                sort!(fs, by= x->cost[v_i, x])
            end
            forward_star[v_i] = fs 
        end
    end
    return forward_star 
end
function save_forward_star!(pg::ESPPRC_Instance; sorted=true)
    n_nodes = length(pg.service_time)
    for v_i in 1:n_nodes
        if v_i == pg.destination
            pg.forward_star[v_i] = Int[]
        else        
            fs = findall(x -> x < Inf, pg.cost[v_i, :])
            if sorted 
                sort!(fs, by= x->pg.cost[v_i, x])
            end
            pg.forward_star[v_i] = fs 
        end
    end
end

function save_reverse_star(pg::ESPPRC_Instance; sorted=true)
    n_nodes = length(pg.service_time)
    return save_reverse_star(n_nodes, pg.cost; sorted=sorted)
end
function save_reverse_star(n_nodes::Int, origin::Int, cost::Matrix{Float64}; sorted=true)
    reverse_star = Vector{Vector{Int}}(undef, n_nodes)
    for v_i in 1:n_nodes
        if v_i == origin
            reverse_star[v_i] = Int[]
        else    
            rs = findall(x -> x < Inf, cost[:, v_i])
            if sorted 
                sort!(rs, by= x->cost[x, v_i])
            end
            reverse_star[v_i] = rs 
        end
    end
    return reverse_star
end
function save_reverse_star!(pg::ESPPRC_Instance; sorted=true)
    n_nodes = length(pg.service_time)
    for v_i in 1:n_nodes
        if v_i == pg.origin
            pg.reverse_star[v_i] = Int[]
        else            
            rs = findall(x -> x < Inf, pg.cost[:, v_i])
            if sorted 
                sort!(rs, by= x->pg.cost[x, v_i])
            end
            pg.reverse_star[v_i] = rs 
        end
    end
end

function graph_reduction!(pg::ESPPRC_Instance)
    n_nodes = length(pg.early_time)
    OD = [pg.origin, pg.destination]

    pg.cost[pg.origin, pg.destination] = Inf

    for j in 1:n_nodes
        pg.cost[j, j] = Inf
        pg.cost[pg.destination, j] = Inf
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
    if v_i == pg.destination
        return false, NaN, NaN
    end

    # Check time 
    arrival_time = max( λ_i.time + pg.service_time[v_i] + pg.time[v_i, v_j] , pg.early_time[v_j] )
    if arrival_time > pg.late_time[v_j]
        return false, NaN, NaN
    end

    # Check capacity
    new_load = λ_i.load + pg.load[v_i, v_j]
    if new_load > pg.capacity
        return false, NaN, NaN
    end

    return true, arrival_time, new_load
end

function backward_reach(λ_i::Label, v_i::Int, v_k::Int, pg::ESPPRC_Instance)
    if v_i == pg.origin
        return false, NaN, NaN
    end

    # Currently at v_i 
    # extending arc, backward: (v_k, v_i)

    a_k = pg.early_time[v_k] + pg.service_time[v_k]
    b_k = pg.late_time[v_k] .+ pg.service_time[v_k]

    # Check time 
    max_T = pg.max_T
    new_time = max(λ_i.time + pg.service_time[v_i] + pg.time[v_k, v_i], max_T - b_k)
    if new_time > max_T - a_k
        return false, NaN, NaN
    end

    # Check capacity
    new_load = λ_i.load + pg.load[v_k, v_i]
    if new_load > pg.capacity
        return false, NaN, NaN
    end

    return true, new_time, new_load
end

function calculate_max_T(pg::ESPPRC_Instance)
    # n_nodes = length(pg.early_time)
    # set_N = 1:n_nodes
    # tmp = [
    #     pg.late_time[i] + pg.service_time[i] + pg.time[i, pg.destination] 
    #     for i in set_N if pg.time[i, pg.destination] < Inf
    # ]
    # max_T = min(maximum(tmp), pg.late_time[pg.destination])
    # return max_T
    calculate_max_T(pg. destination, pg.time, pg.early_time, pg.late_time, pg.service_time)
end
function calculate_max_T(destination, time, early_time, late_time, service_time)
    n_nodes = length(early_time)
    set_N = 1:n_nodes
    tmp = [
        late_time[i] + service_time[i] + time[i, pg.destination] 
        for i in set_N if time[i, destination] < Inf
    ]
    max_T = min(maximum(tmp), late_time[destination])
    return max_T
end


function calculate_path_time(path::Vector{Int}, pg::ESPPRC_Instance; check_feasible=false)
    total_time = 0
    for k in 1:length(path)-1
        i, j = path[k], path[k+1]
        total_time = max(total_time + pg.service_time[i] + pg.time[i, j], pg.early_time[j])
        if check_feasible
            if total_time > pg.late_time[j] 
                @warn("This path is infeasible at node $j.")
                show_details(path, pg)
                readline()
            end
        end
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

function deci3(n::Float64)
    return round(n, digits=3)
end
function deci1(n::Float64)
    return round(n, digits=1)
end




function show_details(path, pg::ESPPRC_Instance)
    println("--------------Path Details ----------------------")
    println(path)
    if length(path) == 0
        return 
    end
    
    j = path[1]
    arr_time = 0.0
    load = 0.0
    cost = 0.0
    cost_change = 0.0
    println("At node $j: time=$(deci3(arr_time)), load=$(deci1(load)), cost=$(deci3(cost))")
    for k in 2:length(path)
        i, j = path[k-1], path[k]
        arr_time = max(arr_time + pg.service_time[i] + pg.time[i,j], pg.early_time[j]) |> deci3
        load += pg.load[i,j] |> deci1
        cost += pg.cost[i,j] |> deci3
        println("At node $j: time=$(deci3(arr_time)), load=$(deci1(load)), cost=$(deci3(cost)), arc_cost=$(pg.cost[i,j])")
        if arr_time > pg.late_time[j]
            @info("Time window constraint is violated at node $j: $(arr_time) > $(pg.late_time[j])")
        end
        if load > pg.capacity
            @info("Capacity constraint is violated at node $j: $(load) > $(pg.capacity)")
        end
    end      
    println("-"^50)  
end


function print_pct_neg_arcs(pg::ESPPRC_Instance) 
    count = 0
    neg = 0 
    n_nodes = length(pg.service_time)
    for i in 1:n_nodes 
        for j in 1:n_nodes
            if i != j && pg.cost[i,j] < Inf && i!=pg.origin
                count += 1
                if pg.cost[i,j] < 0.0 
                    neg += 1
                end
            end
        end
    end

    println("% neg arcs : ", round(neg/count * 100, digits=2), " %")
end