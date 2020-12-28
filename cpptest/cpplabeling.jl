using Test 
using Random
using BenchmarkTools
# For Debugging
include("../src/VRPTWinclude.jl")


using Cxx, Libdl 
const path_to_lib = pwd() 
@show path_to_lib
addHeaderDir(path_to_lib, kind=C_System)
Libdl.dlopen(joinpath(path_to_lib, "liblabeling.dylib"), Libdl.RTLD_GLOBAL)
cxxinclude("labeling.h")
cxxinclude("vector")

function matrix_to_vector(mat::Matrix{T}) where T
    arr = Vector{Vector{T}}(undef, size(mat, 1))
    for i in eachindex(arr)
        arr[i] = mat[i, :] :: Vector{T} 
    end
    return arr
end

function solveESPPRC_cpp_labeling(espprc::ESPPRC_Instance)
    pg = deepcopy(espprc)

    # @show(matrix_to_vector(pg.cost))
    # @show(matrix_to_vector(pg.time))
    # @show(matrix_to_vector(pg.load))
    # @show((pg.early_time))
    # @show((pg.late_time))
    # @show((pg.service_time))
    # @show((pg.forward_star))
    # readline()
    
    for i in 1:length(pg.forward_star)
        for j in 1:length(pg.forward_star[i])
            pg.forward_star[i][j] -= 1
        end
    end

    n = length(pg.service_time)
    o = pg.origin - 1
    d = pg.destination - 1
    cap = pg.capacity
    c = convert(cxxt"std::vector<std::vector<double>>", matrix_to_vector(pg.cost))
    t = convert(cxxt"std::vector<std::vector<double>>", matrix_to_vector(pg.time))
    l = convert(cxxt"std::vector<std::vector<double>>", matrix_to_vector(pg.load))
    et = convert(cxxt"std::vector<double>", pg.early_time)
    lt = convert(cxxt"std::vector<double>", pg.late_time)
    st = convert(cxxt"std::vector<double>", pg.service_time)
    fs = convert(cxxt"std::vector<std::vector<int>>", pg.forward_star)

    cppmodel = icxx"LabelingAlgorithm model($n, $o, $d, $cap, $c, $t, $l, $et, $lt, $st, $fs); model;"
    
    cxx_path = icxx"std::vector<int> cxx_path = $cppmodel.monodirectional(); cxx_path;"
    c = icxx" &$cxx_path[0];"
    cSize = icxx" $(cxx_path).size(); "
    j_path = unsafe_wrap(Array, c, cSize)
    return convert(Vector{Int}, j_path) .+= 1
end

solomon_dataset_name = "R102_050"
solomon = load_solomon(solomon_dataset_name)
num_nodes = solomon.nodes |> length
# Random.seed!(123)
dual_var_org = (rand(num_nodes) * 15)
dual_var_pulse = [dual_var_org[2:end]; 0.0; 0.0]
espprc = solomon_to_espprc(solomon, dual_var_pulse)
@time lab_path = solveESPPRC_cpp_labeling(espprc)
@show lab_path 

@time sol = solveESPPRC(espprc, method="pulse")
@show sol.cost
@show sol.path

@time lab2 = solveESPPRC(espprc, method="monodirectional")
@show lab2.cost
@show lab2.path

@show calculate_path_cost(lab_path, espprc.cost)
@show calculate_path_cost(sol.path, espprc.cost)
@show calculate_path_cost(lab2.path, espprc.cost)

@test calculate_path_cost(lab_path, espprc.cost) >= calculate_path_cost(sol.path, espprc.cost)
if lab_path != sol.path
    @warn("Paths are different. But mine is better")
    @show lab_path 
    @show sol.path 
end

println("Done")


