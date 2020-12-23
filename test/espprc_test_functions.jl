

function show_details(path, pg::ESPPRC_Instance)
    println("--------------Path Details ----------------------")
    println(path)
    if length(path) == 0
        return 
    end
    
    j = path[1]
    arr_time = 0.0
    load = 0.0
    cost = 0.0
    cost_change = 0.0
    println("At node $j: time=$(arr_time), load=$(load), cost=$(cost), cost_change=$cost_change")
    for k in 2:length(path)
        i, j = path[k-1], path[k]
        arr_time = max(arr_time + pg.service_time[i] + pg.time[i,j], pg.early_time[j])
        load += pg.load[i,j]
        cost += pg.cost[i,j]
        cost_change = pg.cost[i,j]
        println("At node $j: time=$(arr_time), load=$(load), cost=$(cost), cost_change=$cost_change")
        if arr_time > pg.late_time[j]
            @info("Time window constraint is violated at node $j: $(arr_time) > $(pg.late_time[j])")
        end
        if load > capacity
            @info("Capacity constraint is violated at node $j: $(load) > $(pg.capacity)")
        end
    end      
    println("-"^50)  
end

