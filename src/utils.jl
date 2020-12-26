

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


function is_binary(y::Vector{Float64})
    tol = 1e-6
    for i in eachindex(y)
        if ! ( isapprox(y[i], 1.0, atol=tol) || isapprox(y[i], 0.0, atol=tol) )
            return false
        end
    end
    return true
end
