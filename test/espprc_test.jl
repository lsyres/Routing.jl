
include("../examples/espprc_example.jl")

@info("Testing for OD=($(pg.origin), $(pg.destination))")

print("Pulse     : "); @time pulse = solveESPPRC(pg, method="pulse")
print("Mono      : "); @time mono0= solveESPPRC(pg, method="monodirectional")
print("Mono DSSR : "); @time mono1 = solveESPPRC(pg, method="monodirectional", DSSR=true)
# print("Bi        : "); @time bidi0= solveESPPRC(pg, method="bidirectional")
# print("Bi   DSSR : "); @time bidi1 = solveESPPRC(pg, method="bidirectional", DSSR=true)

@show pulse.cost, pulse.load, pulse.time
@show mono0.cost, mono0.load, mono0.time
@show mono1.cost, mono1.load, mono1.time
# @show bidi0.cost, bidi0.load, bidi0.time
# @show bidi1.cost, bidi1.load, bidi1.time
@show pulse.path
@show mono0.path
@show mono1.path
# @show bidi0.path
# @show bidi1.path

@testset "ESPPRC Example Test" begin
    @test isapprox(pulse.cost, mono0.cost, atol=1e-7)
    @test isapprox(mono0.cost, mono1.cost, atol=1e-7)
    # @test isapprox(pulse.cost, bidi0.cost, atol=1e-7)
    # @test isapprox(bidi0.cost, bidi1.cost, atol=1e-7)
end



# negative cost route return test 
max_neg = 10
@info("ESPPRC Example testing with max_neg_routes=$(max_neg)...")

@time pulse, neg_pulse = solveESPPRC_vrp(pg, method="pulse", max_neg_routes=max_neg)
@time mono0, neg_mono0 = solveESPPRC_vrp(pg, method="monodirectional", max_neg_routes=max_neg)
@time mono1, neg_mono1 = solveESPPRC_vrp(pg, method="monodirectional", max_neg_routes=max_neg, DSSR=true)

@show typeof(pulse), typeof(neg_pulse), length(neg_pulse)
@show pulse 
@show mono0
@show mono1

@testset "ESPPRC Example Test with max_neg_routes" begin
    @test length(neg_pulse) <= max_neg
    @test length(neg_mono0) <= max_neg
    @test length(neg_mono1) <= max_neg
    for l in neg_pulse
        @test l.cost < 0.0
        @test l.path[1] == pg.origin        
        @test l.path[end] == pg.destination
    end
    for l in neg_mono0
        @test l.cost < 0.0
        @test l.path[1] == pg.origin        
        @test l.path[end] == pg.destination        
    end
    for l in neg_mono1
        @test l.cost < 0.0
        @test l.path[1] == pg.origin        
        @test l.path[end] == pg.destination        
    end
end