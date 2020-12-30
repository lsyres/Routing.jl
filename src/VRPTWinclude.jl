using LightXML
using JuMP, GLPK
using ElasticArrays
using DataStructures

global counter = 0
global counter1 = 0
global counter2 = 0
const EPS = 1e-9
const MAX_INT = 9223372036854775807
const DIGITS = 9

include("data_structure.jl")
include("utils.jl")
include("solomon.jl")
include("ESPPRC.jl")
include("pulse.jl")
include("labeling.jl")
include("relaxed_master_problem.jl")
include("branch_and_bound.jl")

