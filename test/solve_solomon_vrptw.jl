using VRPTW

using PyPlot 


struct SolomonDataset
    vrptw::VRPTW_Instance
    data_name::String
    nodes::Vector{Node}
    fleet::Fleet
    requests::Vector{Request}
end

function generate_solomon_vrptw_instance(dataset_name)
    data_file_path = dataset_name * ".xml"
    data_file_path = joinpath(@__DIR__, "..", "solomon-1987", data_file_path)
    dataset, data_name, nodes, fleet, requests = read_solomon_data(data_file_path)

    n_nodes = length(nodes)
    n_vehicles = fleet.number
    n_customers = length(requests)

    # Add a dummy node for depot at the end
    push!(nodes, nodes[n_nodes])
    depot0 = n_customers + 1
    depot_dummy = n_customers + 2
    cost = calculate_cost(nodes)
    cost[depot0, depot_dummy] = Inf
    cost[depot_dummy, depot0] = Inf

    n_nodes = length(nodes)
    @assert n_nodes - 2 == n_customers
    V = 1:n_vehicles
    N = 1:n_nodes
    C = 1:n_customers

    d = [requests[i].quantity for i in C]
    q = fleet.capacity
    a = [requests[i].start_time for i in C]
    b = [requests[i].end_time for i in C]
    t = copy(cost)
    θ = [requests[i].service_time for i in C]

    vrptw_inst = VRPTW_Instance(
        t,
        θ,
        a,
        b,
        d,
        q,
        fleet.max_travel_time
    )

    return SolomonDataset(vrptw_inst, data_name, nodes, fleet, requests)
end


function plot_solomon_solution(solomon::SolomonDataset, sol_y, sol_routes, sol_obj, duration)
    data_name = solomon.data_name
    nodes = solomon.nodes

    n_nodes = length(nodes)
    N = 1:n_nodes

    fig = figure() 
    for i in N 
        plot(nodes[i].cx, nodes[i].cy, marker=".", color="r", label=i)
        annotate(i, (nodes[i].cx, nodes[i].cy), textcoords="offset points", xytext=(2,2), ha="center")
    end
    for k in 1:length(sol_y)
        if sol_y[k] > 1 - 1e-6
            r = sol_routes[k]
            for n in 1:length(r)-1
                i, j = r[n], r[n + 1]
                # plot([node[i].cx, node[j].cx], [node[i].cy, node[j].cy], color="r")
                dx = nodes[j].cx - nodes[i].cx
                dy = nodes[j].cy - nodes[i].cy
                arrow(nodes[i].cx, nodes[i].cy, dx, dy, 
                    color="b", head_width=0.5, length_includes_head=true)
            end
        end
    end
    sol_obj = round(sol_obj * 100) / 100
    duration = round(duration * 100) /100
    title("$(data_name) by Algorithm: obj=$(sol_obj), duration=$(duration) s")
    savefig("$(data_name)-Algo.png", dpi=1000)
    close(fig)
end



# solomon_vrptw = generate_solomon_vrptw_instance("RC102_025")

solomon = generate_solomon_vrptw_instance("R102_025")

start_time = time()
@time sol_y, sol_routes, sol_obj = solve_vrp_bnb(solomon.vrptw, tw_reduce=false);
end_time = time()
duration = end_time - start_time

plot_solomon_solution(solomon, sol_y, sol_routes, sol_obj, duration)


# @show sol_obj
# for n in 1:length(sol_y)
#     if sol_y[n] > 0.01
#         @show n, sol_y[n], sol_routes[n]
#     end
# end