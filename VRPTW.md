## Vehicle Routing Problem with Time Windows (VRPTW)

### VRPTW Input 

You can construct `Solomon`.

### `Solomon`

To create `Solomon`:

```julia
struct Solomon
    data_name::String
    nodes::Vector{Node}
    fleet::Fleet
    requests::Vector{Request}
end
```
Each component also utilizes the following types:
```julia
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
```

This format is consistent with the format used by [VRP-REP.org](http://www.vrp-rep.org/search.html?slug=solomon) for the Solomon dataset, originally proposed in:

- [Solomon, M.M., 1987. Algorithms for the vehicle routing and scheduling problems with time window constraints. Operations research, 35(2), pp.254-265.](https://doi.org/10.1287/opre.35.2.254)

See [this example](https://github.com/chkwon/Routing.jl/blob/master/examples/vrptw_example.jl) and [that example](https://github.com/chkwon/Routing.jl/blob/master/examples/vrptw_example_with_dists.jl)


### Solomon Benchmark Instances

For an example of loading and solving the Solomon instances, see [test/solve_solomon_vrptw.jl](https://github.com/chkwon/VRPTW.jl/blob/master/test/solve_solomon_vrptw.jl) 

<img src="https://github.com/chkwon/VRPTW.jl/raw/master/R102_025.png" width=500>

The distance between two coordinates is calculated by
```julia
dist = floor(10 * sqrt( (x1-x2)^2 + (y1-y2)^2 )) / 10
```
as described in [this paper](https://doi.org/10.1287/trsc.33.1.101)




### VRPTW Algorithm 

Currently, this package implements a Branch-and-Price algorithm. The subproblem is solved as an *elementary* shortest path problem with resource constraints (ESPPRC). See [this book chapter](https://epubs.siam.org/doi/10.1137/1.9781611973594.ch5) for general description. LP relaxation in each branch is solved by [GLPK.jl](https://github.com/jump-dev/GLPK.jl) via [JuMP.jl](https://github.com/jump-dev/JuMP.jl). 

- Branch-and-Price: [Desrochers, M., Desrosiers, J. and Solomon, M., 1992. A new optimization algorithm for the vehicle routing problem with time windows. Operations research, 40(2), pp.342-354.](https://doi.org/10.1287/opre.40.2.342)

#### Features
* Branch
  - [x] branching on number of vehicles
  - [x] branching on each arc flow

* Cut
  - [ ] valid inequalities ... 

* Price
  - [x] ESPPRC by Pulse Algorithm
  - [x] ESPPRC by labeling algorithm
  - [x] First 400 columns with negative reduced costs were added. 
  - [ ] Approximate methods? 



### Calling from Python 

You can call `solveVRPpy` from Python via [`pyjulia`](https://github.com/JuliaPy/pyjulia).

1. First install this pakcage `Routing.jl` in your Julia.
2. Install `pyjulia` in your Python environment, following the [instruction](https://github.com/JuliaPy/pyjulia)). Note: when you run `julia.install()`, it will connect your current Python to PyCall.jl in your Julia. If you have used PyCall.jl within Julia, it may disconnect the old link. 
3. Try [this example](https://github.com/chkwon/Routing.jl/blob/master/examples/vrptw_example.py) and [that example](https://github.com/chkwon/Routing.jl/blob/master/examples/vrptw_example_with_dists.py)
