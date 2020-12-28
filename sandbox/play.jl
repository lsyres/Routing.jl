using BenchmarkTools


function sum_row(A::Matrix{Float64})
    sum = 0.0
    for i in 1:size(A,1)
        for j in 1:size(A,2)
            sum += A[i, j]
        end
    end
    return sum
end

function sum_col(A::Matrix{Float64})
    sum = 0.0
    for j in 1:size(A,2)
        for i in 1:size(A,1)
            sum += A[i, j]
        end
    end
    return sum
end

function sum_vec_row(A::Vector{Vector{Float64}})
    sum = 0.0
    for i in 1:length(A)
        for j in 1:length(A[i])
            sum += A[i][j]
        end
    end
    return sum
end

function sum_vec_col(A::Vector{Vector{Float64}})
    sum = 0.0
    for j in 1:length(A[1])
        for i in 1:length(A)
            sum += A[i][j]
        end
    end
    return sum
end

n = 1000
A = rand(n,n)

B = Vector{Vector{Float64}}(undef, n)
for i in 1:n
    B[i] = A[i,:]
end

@btime sum_row(A)
@btime sum_col(A)
@btime sum(A) 

@btime sum_vec_row(B)
@btime sum_vec_col(B)

@assert sum_row(A) == sum_vec_row(B) 
