[![Build Status](https://travis-ci.org/chkwon/VRPTW.jl.svg?branch=master)](https://travis-ci.org/chkwon/VRPTW.jl)
[![Coverage Status](https://coveralls.io/repos/github/chkwon/VRPTW.jl/badge.svg?branch=master)](https://coveralls.io/github/chkwon/VRPTW.jl?branch=master)

# VRPTW.jl

*Work in progress...*

This package implements a branch-and-price algorithm for solving Vehicle Routing Problems with Time Windows (VRPTW), where pricing subproblems are solved as Elementary Shortest Path Problems with Resource Constraints (ESPPRC).


## Installation

```julia
] add https://github.com/chkwon/VRPTW.jl
```


## Vehicle Routing Problem with Time Windows (VRPTW)

### VRPTW Input 

You can either construct `VRPTW_Instance` or `SolomonDataset`.

### `VRPTW_Instance`

```julia
struct VRPTW_Instance
    travel_time     ::Matrix{Float64}
    service_time    ::Vector{Float64}
    early_time      ::Vector{Float64}
    late_time       ::Vector{Float64}
    load            ::Vector{Float64}
    capacity        ::Float64
    max_travel_time ::Float64
end
```
The generation of `VRPTW_Instance` is described in this example: [`test/vrptw_example.jl`](https://github.com/chkwon/VRPTW.jl/blob/master/test/vrptw_example.jl).

### `SolomonDataset`

Another way is to create `SolomonDataset`.
```julia
struct SolomonDataset
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

One important difference is that the node id for the depot is the largest among all nodes, instead of 0. The node id numbering begins with 1.

For an example of loading and solving the Solomon instances, see [test/solomon_example.jl](https://github.com/chkwon/VRPTW.jl/blob/master/test/solomon_example.jl) 

<img src="https://github.com/chkwon/VRPTW.jl/raw/master/R102_025.png" width=500>


When `SolomonDataset` is used, the distance between two coordinates is calculated by
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
  - [x] ESPPRC by labeling (monodirectional, bidirectional) algorithm
  - [x] First 400 columns with negative reduced costs were added. 
  - [ ] Approximate methods? 

## Elementary Shortest Path Problem with Resource Constraints (ESPPRC)

This problem seeks to find an elementary path (without cycles) to minimize the total costs considering two resources: load and time. The total load must be less than or equals to the vehicle capacity, and the arrival time in each node must be no later than the latest arrival time. If the vehicle arrives earlier than the earliest arrival time, then the vehicle should wait.

While ESPPRC is used mostly in the VRP context, it is also useful in some other applications; for example, for finding the most risky path with a travel time threshold, as in [A. Bogyrbayeva*, C. Kwon. Pessimistic Evasive Flow Capturing Problems. European Journal of Operational Research, to appear.](https://www.chkwon.net/papers/bogyrbayeva_pessimistic.pdf)

### ESPPRC Input 

You need to generate `ESPPRC_Instance`:
```julia
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
```

See this example for details: [`test/espprc_example.jl`](https://github.com/chkwon/VRPTW.jl/blob/master/test/espprc_example.jl).

### ESPPRC Algorithm

This package implements the following algorithms:

- The Pulse algorithm, as introduced in: [Leonardo Lozano, Daniel Duque, Andrés L. Medaglia (2016) An Exact Algorithm for the Elementary Shortest Path Problem with Resource Constraints. Transportation Science 50(1):348-357.](https://doi.org/10.1287/trsc.2014.0582)

  - Callable by `solveESPPRC(problem::ESPPRC_Instance, method="pulse")`
  - If you need a faster c++ implementation for this algorithm, I recommend https://github.com/DouYishun/vrp-espprc.

- A monodirectional dynamic programming method, as described in: [Feillet, D., Dejax, P., Gendreau, M., Gueguen, C., 2004. An exact algorithm for the elementary shortest path problem with resource constraints: Application to some vehicle routing problems. Networks 44, 216–229](https://onlinelibrary.wiley.com/doi/abs/10.1002/net.20033)
  - Callable by `solveESPPRC(problem::ESPPRC_Instance, method="monodirectional")`
  - Currently, the reachability concept is not implemented.

- A bidirectional dynamic programming method: [Righini, G., Salani, M., 2006. Symmetry helps: Bounded bi-directional dynamic programming for the elementary shortest path problem with resource constraints. Discrete Optimization 3, 255–273.](https://doi.org/10.1016/j.disopt.2006.05.007)
  - Callable by `solveESPPRC(problem::ESPPRC_Instance, method="bidirectional")`

