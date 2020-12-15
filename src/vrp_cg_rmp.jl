using JuMP, GLPK
using ElasticArrays

include("pulse.jl")

struct VRPTW_Instance
    travel_time     ::Matrix{Float64}
    service_time    ::Vector{Float64}
    early_time      ::Vector{Float64}
    late_time       ::Vector{Float64}
    load            ::Vector{Float64}
    capacity        ::Float64
    max_travel_time ::Float64
end

function read_vrptw_instance(vrptw::VRPTW_Instance)
    @assert size(vrptw.travel_time, 1) == size(vrptw.travel_time, 2)
    n_nodes = size(vrptw.travel_time, 1)

    @assert length(vrptw.service_time) == length(vrptw.early_time) == length(vrptw.late_time) == length(vrptw.load)
    n_customers = length(vrptw.service_time)

    @assert n_nodes == n_customers + 2

    depot0 = n_customers + 1
    depot_dummy = n_customers + 2
    @assert vrptw.travel_time[depot0, depot_dummy] == Inf
    @assert vrptw.travel_time[depot_dummy, depot0] == Inf

    return n_nodes, n_customers, depot0, depot_dummy
end



function solve_cg_rmp(vrptw::VRPTW_Instance; optimizer = GLPK.Optimizer)
    n_nodes, n_customers, depot0, depot_dummy = read_vrptw_instance(vrptw)
    set_N = 1:n_nodes
    set_C = 1:n_customers

    # add service time to the cost matrix
    cost_mtx = vrptw.travel_time
    for i in set_N, j in set_N
        if i in set_C
            cost_mtx[i, j] = cost_mtx[i, j] + vrptw.service_time[i]
        else 
            cost_mtx[i, j] = cost_mtx[i, j]
        end
    end

    origin = depot0
    destination = depot_dummy
    capacity = vrptw.capacity

    resrc_mtx = zeros(n_nodes, n_nodes)
    for i in set_N, j in set_N
        if j in set_C
            resrc_mtx[i, j] = vrptw.load[j]
        end
    end

    early_time = [vrptw.early_time; 0; 0]
    late_time = [vrptw.late_time; 0; vrptw.max_travel_time]
    service_time = [vrptw.service_time; 0; 0]
    load = [vrptw.load; 0; 0]



    function calcuate_cost_routes(new_route)
        route_cost = 0.0
        for i in 1:length(new_route)-1
            route_cost += cost_mtx[new_route[i], new_route[i+1]]
        end
        return route_cost
    end

    function add_route!(routes, cost_route, incidence, new_route)
        push!(routes, new_route)
        push!(cost_route, calcuate_cost_routes(new_route))
        customers = new_route[2:end-1]
        inc = zeros(Int, n_customers)
        inc[customers] .= 1 
        append!(incidence, inc)    
    end

    routes = []
    cost_routes = []
    incidence = ElasticArray{Float64}(undef, n_customers, 0)
    for c in set_C
        new_route = [depot0, c, depot_dummy]
        add_route!(routes, cost_routes, incidence, new_route)
    end

    sol_routes = []
    sol_y = []
    sol_obj = Inf
    max_iter = 10000

    # gurobi_env = Gurobi.Env()

    function solve_RMP(routes, cost_routes, incidence; optimizer=GLPK.Optimizer)
        RMP = Model(GLPK.Optimizer)
        set_R = 1:length(routes)
        set_C = 1:size(incidence, 1)
        @variable(RMP, y[set_R] >= 0)
        @objective(RMP, Min, sum(cost_routes[r] * y[r] for r in set_R))
        @constraint(RMP, rrc[i in set_C], sum(incidence[i, r] * y[r] for r in set_R) == 1)
        optimize!(RMP)
        return JuMP.dual.(rrc), JuMP.value.(y), objective_value(RMP)
    end


    for iter in 1:max_iter        
        dual_var, sol_y, sol_obj = solve_RMP(routes, cost_routes, incidence; optimizer=optimizer)
        # α[depot0] is for the depot
        # depot0 is the origin, depot_dummy is the destination
        α = [dual_var; 0.0; 0.0]
        
        @info("---- iteration $iter -----(n_routes = $(length(routes))-----(obj = $sol_obj)---------------")

        # Create a new cost_mtx 
        cost_mtx_copy = copy(cost_mtx)
        for i in set_N, j in set_N 
            if i in set_C
                cost_mtx_copy[i, j] -= α[i]
            end
        end
        time_mtx = copy(cost_mtx)

        # solve ESPPRC

        pg = PulseGraph(
            origin,
            destination,
            capacity,
            cost_mtx_copy,
            time_mtx,
            resrc_mtx,
            early_time,
            late_time,
        )

        dp_state = solveESPPRCpulse(pg)

        if ! in(dp_state.path, routes) && dp_state.cost < - 1e-6
            add_route!(routes, cost_routes, incidence, dp_state.path)
            sol_routes = routes   
        else 
            println("--- no new route is added. ---")
            sol_routes = routes           
            if !isapprox(dp_state.cost, 0, atol=1e-6)        
                @warn("The reduced_cost is negative: ", dp_state.cost)
            end
            break
        end
        println("New path: ", dp_state.path)
        println("Reduced cost: ", dp_state.cost)

    end

    return sol_y, sol_routes, sol_obj

end

