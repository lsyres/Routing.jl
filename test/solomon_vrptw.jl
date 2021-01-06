# using Routing
include("../src/Routing_include.jl")

# To plot
using PyPlot

function plot_solomon_solution(solomon::Solomon, sol_routes, sol_obj, duration)
    data_name = solomon.data_name
    nodes = solomon.nodes

    n_nodes = length(nodes)
    N = 1:n_nodes

    fig = figure() 
    for i in N 
        plot(nodes[i].cx, nodes[i].cy, marker=".", color="r", label=i)
        annotate(i, (nodes[i].cx, nodes[i].cy), textcoords="offset points", xytext=(2,2), ha="center")
    end
    for r in sol_routes
        for n in 1:length(r)-1
            i, j = r[n], r[n + 1]
            dx = nodes[j].cx - nodes[i].cx
            dy = nodes[j].cy - nodes[i].cy
            arrow(nodes[i].cx, nodes[i].cy, dx, dy, 
                color="b", head_width=0.5, length_includes_head=true)
        end
    end
    sol_obj = round(sol_obj * 100) / 100
    duration = round(duration * 100) /100
    title("$(data_name): obj=$(sol_obj), duration=$(duration) s")
    savefig("$(data_name).png", dpi=1000)
    close(fig)
end

# solomon_vrptw = generate_solomon_vrptw_instance("RC102_025")

solomon_dataset_name = "R102_025"

solomon = load_solomon(solomon_dataset_name)
vrptw = generate_solomon_vrptw_instance(solomon)

start_time = time()
@time routes, obj_val = solveVRP(vrptw, pricing_method="monodirectional");
end_time = time()
duration = end_time - start_time

@show obj_val
@show routes

plot_solomon_solution(solomon, routes, obj_val, duration)
@show solomon_dataset_name, obj_val, duration


include("solomon_exact.jl")
solomon_exact(vrptw)
