using JuMP, Gurobi

function solomon_exact(vrptw::VRPTW_Instance)
    n_nodes = size(vrptw.travel_time, 1)
    n_requests = length(vrptw.service_time)
    n_vehicles = n_requests 
    @show n_nodes, n_requests
    @assert n_nodes - 2 == n_requests

    V = 1:n_vehicles
    N = 1:n_nodes
    C = 1:n_requests
    depot0 = n_requests + 1
    depot_dummy = n_requests + 2

    d = vrptw.load
    q = vrptw.capacity
    a = vrptw.early_time
    b = vrptw.late_time
    cost = copy(vrptw.travel_time)
    for i in N, j in N 
        if cost[i, j] == Inf 
            cost[i, j] = 0.0 
        end 
    end    
    
    t = copy(vrptw.travel_time)
    for i in N, j in N 
        if i in C
            t[i, j] = t[i, j] + vrptw.service_time[i]
        else 
            t[i, j] = t[i, j]
        end
        if t[i, j] == Inf 
            t[i, j] = 0.0
        end
    end

    a = [a; 0; 0]
    b = [b; 0; vrptw.max_travel_time]
    bigM = vrptw.max_travel_time * 2
    bigQ = q + maximum(d)

    m = nothing 
    x = nothing 
    s = nothing

    m = Model(Gurobi.Optimizer)
    @variable(m, x[N, N], Bin)
    @variable(m, s[N] >= 0)
    @variable(m, qq[N] >= 0)

    @objective(m, Min, sum(cost[i,j] * x[i,j] for i in N, j in N))
    
    @constraint(m, [i in C], sum(x[i,j] for j in N) == 1 )
    
    @constraint(m, [i in N, j in C], qq[j] >= qq[i] + d[j] - bigQ * (1 - x[i,j]))
    @constraint(m, [i in N], 0 <= qq[i] <= q)
    
    @constraint(m, sum(x[depot0,j] for j in N) <= n_vehicles )
    @constraint(m, [h in C], sum(x[i,h] for i in N) - sum(x[h,j] for j in N) == 0 )
    @constraint(m, sum(x[i,depot_dummy] for i in N) == sum(x[depot0,j] for j in N) )

    @constraint(m, [i in N, j in N], s[j] >= s[i] + t[i,j] - bigM * (1 - x[i,j]))
    @constraint(m, [i in N], a[i] <= s[i] <= b[i])
    
    @constraint(m, [i in N], x[i, depot0] == 0)
    @constraint(m, [i in N], x[depot_dummy, i] == 0)
    @constraint(m, [i in N], x[i, i] == 0)
    
    start_time = time()
    optimize!(m)
    duration = time() - start_time

    @show raw_status(m)
    @show objective_value(m)


    x_val = JuMP.value.(x).data
    first_nodes = findall(x->x>0.9, x_val[depot0, :])
    route_costs = Dict()
    for i in first_nodes
        next = findfirst(x->x>0.9, x_val[i, :])
        route = [depot0, i, next]
        cc = cost[depot0, i] + cost[i, next]
        while next != depot_dummy
            current = next
            next = findfirst(x->x>0.9, x_val[current, :])
            push!(route, next)
            cc += cost[current, next]
            # @show route
        end
        route_costs[route] = cc
    end
    display(route_costs)
    @show sum([route_costs[s] for s in keys(route_costs)])
    println(route_costs)
end