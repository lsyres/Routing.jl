# common functions for ESPPRC 
# pulse.jl and labeling.jl 


# Main ESPPRC function
function solveESPPRC(org_pg::ESPPRC_Instance; max_neg_cost_routes=Inf, method="pulse", DSSR=false)
    if method == "pulse"
        return solveESPPRCpulse(org_pg; max_neg_cost_routes=max_neg_cost_routes)
    elseif method == "monodirectional"
        return monodirectional(org_pg; max_neg_cost_routes=max_neg_cost_routes, DSSR=DSSR)
    else
        return bidirectional(org_pg; max_neg_cost_routes=max_neg_cost_routes, DSSR=DSSR)
    end
end




function graph_reduction!(pg::ESPPRC_Instance)
    n_nodes = length(pg.early_time)
    OD = [pg.origin, pg.destination]

    for i in 1:n_nodes, j in 1:n_nodes
        if i != j && !in(i, OD) && !in(j, OD)
            if pg.early_time[i] + pg.service_time[i] + pg.time[i, j] > pg.late_time[j]
                pg.cost[i, j] = Inf
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
