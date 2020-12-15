include("vrp_cg_rmp.jl")


# branch and bound

function is_binary(y)
    tol = 1e-6
    for i in eachindex(y)
        if ! ( isapprox(y[i], 1.0, atol=tol) || isapprox(y[i], 0.0, atol=tol) )
            return false
        end
    end
    return true
end

function solve_vrp_bnb(vrptw::VRPTW_Instance)

    # solve root node 
    sol_y, sol_routes, sol_obj = solve_cg_rmp(vrptw, initial_routes=[])

    @show is_binary(sol_y)


    return sol_y, sol_routes, sol_obj
end
