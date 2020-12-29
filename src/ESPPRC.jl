# common functions for ESPPRC 
# pulse.jl and labeling.jl 


# Main ESPPRC function
function solveESPPRC(org_pg::ESPPRC_Instance; method="pulse", DSSR=false)
    if method == "pulse"
        best_label, _ = solveESPPRCpulse(org_pg)
    elseif method == "monodirectional"
        best_label, _ = monodirectional(org_pg)
    elseif method == "bidirectional"
        best_label, _ = bidirectional(org_pg; DSSR=DSSR)
    else 
        best_label, _ = solveESPPRCpulse(org_pg)
    end

    return best_label
end

function solveESPPRC_vrp(org_pg::ESPPRC_Instance; max_neg_routes=MAX_INT, method="pulse", DSSR=false)
    if method == "pulse"
        return solveESPPRCpulse(org_pg; max_neg_routes=max_neg_routes)
    elseif method == "monodirectional"
        return monodirectional(org_pg; max_neg_routes=max_neg_routes, DSSR=DSSR)
    elseif method == "bidirectional"
        return bidirectional(org_pg; max_neg_routes=max_neg_routes, DSSR=DSSR)
    else 
        return solveESPPRCpulse(org_pg; max_neg_routes=max_neg_routes)
    end
end
