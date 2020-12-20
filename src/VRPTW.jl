# __precompile__(false)

module VRPTW

using LightXML
using JuMP, GLPK
using ElasticArrays
using DataStructures

include("data_structure.jl")
include("read_solomon_data.jl")
include("pulse.jl")
include("vrp_cg_rmp.jl")
include("vrp_bnb.jl")
include("solomon.jl")

export Pulse, PulseGraph, VRPTW_Instance, 
        Node, Fleet, Request, SolomonDataset,
        solveESPPRCpulse, solve_vrp_bnb,
        read_solomon_data, calculate_solomon_cost,
        load_solomon, generate_solomon_vrptw_instance


end # module