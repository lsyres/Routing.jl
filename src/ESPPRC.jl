# common functions for ESPPRC 
# pulse.jl and labeling.jl 


# Main ESPPRC function
function solveESPPRC(org_pg::ESPPRC_Instance; max_neg_cost_routes=MAX_INT, method="pulse", DSSR=false)
    if method == "pulse"
        return solveESPPRCpulse(org_pg; max_neg_cost_routes=max_neg_cost_routes)
    elseif method == "monodirectional"
        return monodirectional(org_pg; max_neg_cost_routes=max_neg_cost_routes, DSSR=DSSR)
    elseif method == "bidirectional"
        return bidirectional(org_pg; max_neg_cost_routes=max_neg_cost_routes, DSSR=DSSR)
    else 
        return solveESPPRCpulse(org_pg; max_neg_cost_routes=max_neg_cost_routes)
    end
end


