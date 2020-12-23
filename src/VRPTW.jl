# __precompile__(false)

module VRPTW

using LightXML
using JuMP, GLPK
using ElasticArrays
using DataStructures

include("data_structure.jl")
include("solomon.jl")
include("pulse.jl")
include("relaxed_master_problem.jl")
include("branch_and_bound.jl")

export Pulse, Label, ESPPRC_Instance, VRPTW_Instance, 
        Node, Fleet, Request, SolomonDataset,
        solveESPPRCpulse, solve_vrp_bnb,
        read_solomon_data, calculate_solomon_cost, calculate_path_time
        load_solomon, generate_solomon_vrptw_instance


end # module