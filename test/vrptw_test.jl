
include("../examples/vrptw_example.jl")

function check_route(r)
    println("------- Route: $r -------------------------------------------------")
    arrival_time = 0 
    total_load = 0
    total_dist = 0
    for k in 1:length(r)-1
        i, j = r[k], r[k+1]
        if j == depot_dummy
            tw = (0, max_travel_time)
            arrival_time = arrival_time + travel_time[i,j]
            arrival_time = max(tw[1], arrival_time)
            total_dist += travel_time[i, depot_dummy]
        else
            tw = time_windows[j]
            arrival_time = arrival_time + travel_time[i,j]
            arrival_time = max(tw[1], arrival_time)
            total_load += load[j]            
            total_dist += travel_time[i,j]
        end

        println("[Node $j] arrival_time: $(arrival_time) ∈ $(tw), total_load: $(total_load) <= $capacity, total_dist=$(total_dist)")
        @assert arrival_time <= tw[2]
        @assert total_load <= capacity
    end
    println("-------------------------------------------------------------------------------------------")

    return total_dist
end

for m in 1:length(routes)
    r = routes[m]
    total_cost = 0
    total_cost += check_route(r)
    @show total_cost
end

println("Nodes 17 and 18 are the depot.")

# Optimal Solution
total_opt_cost = 0
total_opt_cost += check_route([17, 9, 14, 16, 10, 18])
total_opt_cost += check_route([17, 7, 1, 4, 3, 18])
total_opt_cost += check_route([17, 12, 13, 15, 11, 18])
total_opt_cost += check_route([17, 5, 8, 6, 2, 18])
@show total_opt_cost

@testset "OR-Tools-Example" begin
    @test total_distance == 73.0
end