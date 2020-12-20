
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

