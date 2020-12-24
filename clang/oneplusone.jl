e = [23, 5, 4, 34, 5, 4]

d = Cint[10, 2, 3, 43, 5]
x = ccall((:show, "liboneplusone"), Cint, (Ptr{Vector{Cint}}, Cint), d, length(d))


len = 1000
vec = collect(100.0:-1:1) .* 10
len = length(vec)

ccall((:set_test, "liboneplusone"), Cvoid, (Ptr{Cdouble}, Cint), vec, len)
y = ccall((:get_test, "liboneplusone"), Ptr{Cdouble}, ())
y_julia = unsafe_wrap(Vector{Cdouble}, y, len; own=false)
@show y_julia

mat = Cdouble[
    1 2 3 4;
    10 20 30 40;
    100 200 300 400
] 


display(mat)
n_rows, n_cols = size(mat)

function Base.cconvert(::Type{Ptr{Ptr{Cdouble}}}, x::Matrix{Cdouble})
    Ref{Ptr{Cdouble}}(
        [Ref(x, i) for i in 1:size(x,1)]
    )
end
# ref_mat = [Ref(mat, i) for i in 1:n_cols:length(mat)]

ccall((:set_matrix, "liboneplusone"), Cvoid, (Ptr{Cdouble}, Cint, Cint), mat, n_rows, n_cols)


mat_c = ccall((:get_matrix, "liboneplusone"), Ptr{Ptr{Cdouble}}, ())
@show typeof(mat_c)

mat_rows = unsafe_wrap(Vector{Ptr{Cdouble}}, mat_c, n_rows)
jrows = Vector{Any}(undef, 0)
for row in mat_rows
    push!(jrows, unsafe_wrap(Vector{Cdouble}, row, n_cols))
end
jmat = zeros(n_rows, n_cols)
for i in 1:n_rows, j in 1:n_cols
    jmat[i,j] = jrows[i][j]
end


