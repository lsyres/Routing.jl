using LightXML
using JuMP, GLPK
using ElasticArrays
using DataStructures

global counter = 0
const EPS = 1e-9
const MAX_INT = 2000000000

include("data_structure.jl")
include("utils.jl")
include("solomon.jl")
include("ESPPRC.jl")
include("pulse.jl")
include("labeling.jl")
include("relaxed_master_problem.jl")
include("branch_and_bound.jl")

