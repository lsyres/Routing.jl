################################################
struct VRPTW_Instance
    travel_time     ::Matrix{Float64}
    service_time    ::Vector{Float64}
    early_time      ::Vector{Float64}
    late_time       ::Vector{Float64}
    load            ::Vector{Float64}
    capacity        ::Float64
    max_travel_time ::Float64
end
################################################



################################################
# ESPPRC algorithm 
################################################

mutable struct Label
    time        ::Float64
    load        ::Float64
    flag        ::Vector{Int}
    cost        ::Float64
    path        ::Vector{Int}
    function Label(time::Float64, load::Float64, flag::Vector{Int}, cost::Float64, path::Vector{Int}) 
        time = round(time, digits=DIGITS)
        load = round(load, digits=DIGITS)
        cost = round(cost, digits=DIGITS)
        new(time, load, flag, cost, path)
    end
end

mutable struct ESPPRC_Instance
    origin      :: Int64
    destination :: Int64
    capacity    :: Float64
    cost        :: Matrix{Float64}
    time        :: Matrix{Float64}
    load        :: Matrix{Float64}
    early_time  :: Vector{Float64}
    late_time   :: Vector{Float64}
    service_time:: Vector{Float64}
    forward_star    :: Vector{Vector{Int}}
    reverse_star    :: Vector{Vector{Int}}
    critical_nodes  :: Set{Int}
    max_T           :: Float64
    max_neg_routes  :: Int
end
function ESPPRC_Instance(origin, destination, capacity, cost, time, load, early_time, late_time, service_time)
    n_nodes = length(service_time)
    fs = save_forward_star(n_nodes, destination, cost; sorted=true)
    rs = save_reverse_star(n_nodes, origin, cost; sorted=true)
    critical_nodes = Set(1:n_nodes)
    max_T = calculate_max_T(destination, time, early_time, late_time, service_time)
    max_neg_routes = MAX_INT
    return ESPPRC_Instance(origin, destination, capacity, cost, time, load, early_time, late_time, service_time, 
                            fs, rs, critical_nodes, max_T, max_neg_routes)
end
################################################

################################################
#  Branch and bounds 
################################################
struct Branch
    history         :: OrderedDict
    vrptw_instance  :: VRPTW_Instance
    root_routes     :: Array
    branch_priority :: Array
    lower_bound     :: Float64
    veh_cond        :: Tuple{String, Int64}
end

mutable struct BestIncumbent
    y::Vector{Float64}
    routes::Vector{Vector{Int}}
    objective::Float64
end

################################################


################################################
# solomon datasets 

struct Node
    id::Int
    # type::Int
    cx::Float64
    cy::Float64
end

struct Fleet 
    # type::Int
    number::Int
    # departure_node::Int
    # arrival_node::Int
    capacity::Float64
    max_travel_time::Float64
end

struct Request
    id::Int
    start_time::Int
    end_time::Int
    quantity::Float64
    service_time::Float64
end

struct Solomon
    nodes::Vector{Node}
    fleet::Fleet
    requests::Vector{Request}
end
################################################



