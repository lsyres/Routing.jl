println("starting.... ")

using JuMP, GLPK

for n in 5:6
    c = rand(n)
    m = Model(GLPK.Optimizer)
    @variable(m, x[1:n] >= 0)
    @objective(m, Min, sum(c[i]*x[i] for i in 1:n))
    @constraint(m, cc, sum(x[i] for i in 1:n) == 1)
    optimize!(m)
    @show raw_status(m)
    @show value.(x), shadow_price(cc), dual(cc)
    @assert shadow_price(cc) == - dual(cc)
end