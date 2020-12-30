

## Elementary Shortest Path Problem with Resource Constraints (ESPPRC)

This problem seeks to find an elementary path (without cycles) to minimize the total costs considering two resources: load and time. The total load must be less than or equals to the vehicle capacity, and the arrival time in each node must be no later than the latest arrival time. If the vehicle arrives earlier than the earliest arrival time, then the vehicle should wait.

While ESPPRC is used mostly in the VRP context, it is also useful in some other applications; for example, for finding the most risky path with a travel time threshold, as in [Bogyrbayeva and Kwon (2021)](https://doi.org/10.1016/j.ejor.2020.12.001).

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

See this example for details: [`examples/espprc_example.jl`](https://github.com/chkwon/Routing.jl/blob/master/examples/espprc_example.jl).

### ESPPRC Algorithm

This package implements the following algorithms:

- The Pulse algorithm, as introduced in: [Leonardo Lozano, Daniel Duque, Andrés L. Medaglia (2016) An Exact Algorithm for the Elementary Shortest Path Problem with Resource Constraints. *Transportation Science* 50(1):348-357.](https://doi.org/10.1287/trsc.2014.0582)

  - Callable by 
  ```julia
  solveESPPRC(problem::ESPPRC_Instance, method="pulse")
  ```

- A monodirectional dynamic programming method, as described in: [Feillet, D., Dejax, P., Gendreau, M., Gueguen, C., 2004. An exact algorithm for the elementary shortest path problem with resource constraints: Application to some vehicle routing problems. *Networks* 44, 216–229](https://onlinelibrary.wiley.com/doi/abs/10.1002/net.20033)
  - Callable by 
  ```julia
  solveESPPRC(problem::ESPPRC_Instance, method="monodirectional")
  ```
  - The decremental state-space relaxation (DSSR) version as proposed by [Righini and Salani (2008)](https://doi.org/10.1002/net.20212) is callable by 
  ```julia
  solveESPPRC(problem::ESPPRC_Instance, method="monodirectional", DSSR=true)
  ```

- (work in progress, not working properly yet) A bidirectional dynamic programming method: [Righini, G., Salani, M., 2006. Symmetry helps: Bounded bi-directional dynamic programming for the elementary shortest path problem with resource constraints. *Discrete Optimization* 3, 255–273.](https://doi.org/10.1016/j.disopt.2006.05.007)
  - Callable by `solveESPPRC(problem::ESPPRC_Instance, method="bidirectional")`



### Calling from Python 

You can call `solveESPPRC` from Python via [`pyjulia`](https://github.com/JuliaPy/pyjulia).

1. Install `pyjulia` in your Python environment, following the [instruction](https://github.com/JuliaPy/pyjulia)). Note: when you run `julia.install()`, it will connect your current Python to PyCall.jl in your Julia. If you have used PyCall.jl within Julia, it may disconnect the old link. 
2. With carefully noting the index differences, especially for `origin` and `destination`, try [this example](https://github.com/chkwon/Routing.jl/blob/master/examples/espprc_example.py).
