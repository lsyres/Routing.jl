# Leonardo Lozano, Daniel Duque, Andrés L. Medaglia (2016) An Exact Algorithm for the Elementary Shortest Path Problem with Resource Constraints. Transportation Science 50(1):348-357. https://doi.org/10.1287/trsc.2014.0582
# Implemented by Changhyun Kwon chkwon@gmail.com


# In the Pulse data structure,
# cost, load, time have been updated for [path; next].
# That is, path is updated by [path; next] later,
# only after all feasbility & pruning checks.
mutable struct Pulse
    path    :: Array{Int64, 1}
    next    :: Int64
    cost    :: Float64 
    load    :: Float64
    time    :: Float64
end

function initialize_pulse(origin; cost=0)
    return Pulse([], origin, cost, 0, 0)
end

function initialize_pulse!(p::Pulse, next; cost=0, path=[])
    p.path = path 
    p.next = next
    p.cost = cost
    p.load = 0
    p.time = 0
end

mutable struct PulseGraph
    origin      :: Int64
    destination :: Int64
    capacity    :: Float64
    cost        :: Matrix{Float64}
    time        :: Matrix{Float64}
    load        :: Matrix{Float64}
    early_time  :: Vector{Float64}
    late_time   :: Vector{Float64}
end

# Δ bound step size 
# [time_lb, time_ub] bounding time limits

function isfeasible(pulse::Pulse, pg::PulseGraph)
    # §4.1 Infeasibility Pruning 
    # if pulse_path is empty, feasible 
    if pulse.path == []
        return true
    end
    
    # - Check cycle
    if in(pulse.next, pulse.path)
        return false
    end

    # - capacity constraint
    if pulse.load > pg.capacity
        return false
    end

    # - time window 
    if pulse.time > pg.late_time[pulse.next]
        return false
    end

    # Otherwise feasible.
    return true

end

function should_rollback(p::Pulse, pg::PulseGraph)
    # Section 4.3 Rollback Pruning
    if length(p.path) < 2
        return false
    end

    v_i = p.path[end-1]
    v_k = p.path[end]
    v_j = p.next

    # p: ...... v_i -> v_k -> v_j
    # pp: ..... v_i -> v_j 

    cost_p = pg.cost[v_i, v_k] + pg.cost[v_k, v_j] 
    cost_pp = pg.cost[v_i, v_j]

    time_p = pg.time[v_i, v_k] + pg.time[v_k, v_j] 
    time_pp = pg.time[v_i, v_j]

    if cost_pp <= cost_p 
        # && time_pp <= time_p
        # dominatedm, should rollback
        # println("*************** dominated!")
        return true
    else
        # non-dominated, no need to rollback
        return false
    end
end




# function solveESPPRC(origin, destination, resrc_cap, cost_mtx, time_mtx, resrc_mtx, tw_mtx; allow_suboptimal=true, route_sets=[])

function solveESPPRCpulse(org_pg::PulseGraph)
    counter = 0 
    
    function graph_reduction!(pg::PulseGraph)
        for i in 1:pg.origin-1, j in 1:pg.origin-1
            if i != j 
                if pg.early_time[i] + pg.time[i, j] > pg.late_time[j]
                    pg.cost[i, j] = Inf
                end
            end
        end
    end

    function bounding_scheme(pg::PulseGraph)
        bounding_counter = 0        
        time_ub = pg.late_time[pg.destination]
        time_lb = 0.1 * time_ub
        Δ = Int(floor((time_ub-time_lb) / 15))
        # Δ = 10
        # @show time_ub, time_lb, Δ

        time_values = collect(time_ub:-Δ:time_lb)
        lower_bounds = fill(-Inf, length(pg.cost[:,1]), length(time_values))
        best_pulse_labels = Array{Pulse, 1}(undef, length(pg.cost[:,1]))
        for i in eachindex(best_pulse_labels)
            best_pulse_labels[i] = initialize_pulse(i; cost=Inf)
        end

        _pg = PulseGraph(1, pg.destination, pg.capacity, pg.cost, pg.time, pg.load, pg.early_time, pg.late_time)
        p = initialize_pulse(1)
        p_star = initialize_pulse(1; cost=Inf)

        bounding_iteration = 0

        for k in 2:length(time_values) 
            for v_i in 1:pg.destination-1
                τ = time_values[k]

                initialize_pulse!(p, v_i)
                p.time = τ 
                p_star = best_pulse_labels[v_i]
                _pg.origin = v_i

                pulse_procedure!(v_i, p, p_star, lower_bounds, time_values, _pg)
                if p_star.path == [] 
                    lower_bounds[v_i, k] = Inf
                else
                    # Thie line below isn't necessary, as we call by reference.
                    # best_pulse_labels[v_i] = deepcopy(p_star) 
                    lower_bounds[v_i, k] = p_star.cost 
                end
                
                bounding_iteration += 1

            end
        end

        return lower_bounds, time_values
    end

    function time_to_time_value_index(t, time_values)
        Δ = time_values[1] - time_values[2] 
        time_ub = time_values[1]
        k = Int(ceil((time_ub - t) / Δ)) + 1
        return k
    end

    function isbounded(p::Pulse, best_p::Pulse, lower_bounds, time_values)
        if isempty(lower_bounds) || isempty(time_values)
            return false
        end


        upper_bound = best_p.cost
        # must use from the bound matrix B the lower closest value to τ available. 
        # time_values (sorted from greatest to smallest)
        # τ <= p.time

        # bit_idx = [time_values .<= p.time]
        # τ = maximum(time_values .* bit_idx[1])
        k = time_to_time_value_index(p.time, time_values)

        if k > length(time_values)
            return false
        else
            bounded = p.cost + lower_bounds[p.next, k] >= upper_bound
            return bounded
        end
    end

    function pulse_procedure!(v_i::Int, p::Pulse, best_p::Pulse, lower_bounds, time_values, pg::PulseGraph)
        # v_i = current node
        @assert v_i == p.next

        bounding_counter += 1

        if p.time < pg.early_time[v_i]
            p.time = pg.early_time[v_i]
        end

        if !isfeasible(p, pg) 
            return
        elseif isbounded(p, best_p, lower_bounds, time_values)
            return
        elseif should_rollback(p, pg)
            return
        end

        counter += 1

        # Update the best route found so far
        if p.next == pg.destination 
            if p.cost < best_p.cost
                best_p.path = p.path
                push!(best_p.path, p.next)
                best_p.next = -1                
                best_p.cost = p.cost 
                best_p.load = p.load
                best_p.time = p.time
            end
            return
        end
            
        # Create a new pulse 
        forward_star = findall(x -> x < Inf, pg.cost[v_i, :])
        for v_j in forward_star
            if v_j != p.next && ! in(v_j, p.path)
                pp = deepcopy(p)
                push!(pp.path, pp.next)
                pp.next = v_j 
                pp.cost += pg.cost[v_i, v_j]
                pp.load += pg.load[v_i, v_j]
                pp.time = max(pg.early_time[v_j], pp.time + pg.time[v_i, v_j]) 
                bounding_inner_counter += 1
                pulse_procedure!(v_j, pp, best_p, lower_bounds, time_values, pg)
            end
        end

        # 
    end


    pg = deepcopy(org_pg)
    graph_reduction!(pg)

    # println("------Bounding scheme start------------------------------------")
    # bounding_start = time()
    bounding_counter = 0  
    bounding_inner_counter = 0  
    # @time 
    lower_bounds, time_values = bounding_scheme(pg)
    # println("Bounding scheme took: $(time()-bounding_start) seconds......")
    # @show bounding_counter, bounding_inner_counter
    # println("------Bounding scheme end------------------------------------")

    p = initialize_pulse(pg.origin)
    best_p = initialize_pulse(pg.origin; cost=Inf)


    # println("------pulse_procedure start------------------------------------")
    # start_pulse = time()
    # @time 
    pulse_procedure!(pg.origin, p, best_p, lower_bounds, time_values, pg)
    # println("Pulse Procedure took: $(time()-start_pulse) seconds......")
    # println("------pulse_procedure end------------------------------------")


    return best_p
end