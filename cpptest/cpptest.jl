using Test 
using Random
using BenchmarkTools
# For Debugging
include("../src/VRPTWinclude.jl")



using Cxx, Libdl 
const path_to_lib = pwd() 
@show path_to_lib
addHeaderDir(path_to_lib, kind=C_System)
Libdl.dlopen(joinpath(path_to_lib, "libespprc.dylib"), Libdl.RTLD_GLOBAL)
cxxinclude("espprc.h")
cxxinclude("vector")

function solomon_to_cpp_data(s::SolomonDataset)
    # int x, y, demand, twl, twu, service_time;
    # the first node is the depot and the last node is also the depot.

    data = Vector{Int32}[]
    for i in 1:length(s.nodes)
        if i == 1 
            id = length(s.nodes)
            x = Int(s.nodes[id].cx)
            y = Int(s.nodes[id].cy)
            demand = 0
            twl = 0
            twu = s.fleet.max_travel_time
            service_time = 0
            tv = Int32[x, y, demand, twl, twu, service_time]
            push!(data, tv)    
        else 
            id = i-1
            x = Int(s.nodes[id].cx)
            y = Int(s.nodes[id].cy)
            demand = Int(s.requests[id].quantity)
            twl = Int(s.requests[id].start_time)
            twu = Int(s.requests[id].end_time)
            service_time = Int(s.requests[id].service_time)
            tv = Int32[x, y, demand, twl, twu, service_time]
            push!(data, tv)    
        end
    end
    return data
end



function solveESPPRC_cpp(solomon, dual_var)
    matrix_data = solomon_to_cpp_data(solomon)

    n_nodes = length(matrix_data)+1
    step = 10
    start = 0
    dest = n_nodes - 1
    ut = matrix_data[1][5]
    lt = 0.1 * ut
    # step = Int(round((ut - lt) / 15))

    max_capacity = solomon.fleet.capacity

    cxx_data = convert(cxxt"std::vector<std::vector<int>>", matrix_data)
    cxx_dual_var = convert(cxxt"std::vector<double>", dual_var)

    cppmodel = icxx"Espprc model($n_nodes, $start, $dest, $step, $lt, $ut, $max_capacity, $cxx_data, $cxx_dual_var); model;"
    
    cxx_path = icxx"std::vector<int> cxx_path = $cppmodel.espprc(); cxx_path;"
    c = icxx" &$cxx_path[0];"
    cSize = icxx" $(cxx_path).size(); "
    j_path = unsafe_wrap(Array, c, cSize)
    return convert(Vector{Int}, j_path) 
end

solomon_dataset_name = "C101_100"
solomon = load_solomon(solomon_dataset_name)
num_nodes = solomon.nodes |> length
# Random.seed!(12332)


dual_var_org = (rand(num_nodes) * 20)
dual_var_cpp = copy(dual_var_org)
dual_var_cpp[1] = 0.0
@time cpp_path = solveESPPRC_cpp(solomon, dual_var_cpp)
@show cpp_path

# readline()

dual_var_pulse = [dual_var_org[2:end]; 0.0; 0.0]
espprc = solomon_to_espprc(solomon, dual_var_pulse)

@time sol = solveESPPRC(espprc, method="pulse")
@show sol.cost
@show sol.path


cpp_path = [espprc.origin; cpp_path[2:end-1]; espprc.destination]

@show calculate_path_cost(cpp_path, espprc.cost)
@show calculate_path_cost(sol.path, espprc.cost)

@test calculate_path_cost(cpp_path, espprc.cost) >= calculate_path_cost(sol.path, espprc.cost)
if cpp_path != sol.path
    @warn("Paths are different. But mine is better")
    @show cpp_path 
    @show sol.path 
end

println("Done")