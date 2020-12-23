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
mutable struct Pulse
    path    :: Array{Int64, 1}
    next    :: Int64
    cost    :: Float64 
    load    :: Float64
    time    :: Float64
end

function Base.show(io::IO, p::Pulse)
    println("Pulse -> path=$(p.path)")
    println("         next=$(p.next)")
    println("         cost=$(p.cost)")
    println("         load=$(p.load)")
    println("         time=$(p.time)")
end

mutable struct Label
    time        ::Float64
    load        ::Float64
    flag        ::Vector{Int}
    cost        ::Float64
    path        ::Vector{Int}
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
    type::Int
    cx::Float64
    cy::Float64
end

struct Fleet 
    type::Int
    number::Int
    departure_node::Int
    arrival_node::Int
    capacity::Float64
    max_travel_time::Float64
end

struct Request
    id::Int
    node::Int
    start_time::Int
    end_time::Int
    quantity::Float64
    service_time::Float64
end

struct SolomonDataset
    data_name::String
    nodes::Vector{Node}
    fleet::Fleet
    requests::Vector{Request}
end

################################################



