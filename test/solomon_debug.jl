# using VRPTW
include("../src/VRPTWinclude.jl")
using Test
using DataStructures

solomon_dataset = OrderedDict(
    # "R101_025" => 617.1, # 1 sec
    # "R102_025" => 547.1, # 2 sec
    # "R103_025" => 454.6, # 2 sec
    # "R104_025" => 416.9, # 3 sec
    # "R105_025" => 530.5, # 1 sec
    # "R106_025" => 465.4, # 8 sec
    # "R107_025" => 424.3, # 2 sec
    # "R108_025" => 397.3, # 5 sec
    # "R109_025" => 441.3, # 1 sec
    # "R110_025" => 444.1, # 30 sec
    # "R111_025" => 428.8, # 13 sec
    # "R112_025" => 393.0, # 15 sec

    # "C101_025" => 191.3, # 1 sec
    # "C102_025" => 190.3, # 29 sec
    # "C103_025" => 190.3, # 51 sec
    # "C104_025" => 186.9, # 150 sec
    # "C105_025" => 191.3, # 1 sec
    # "C106_025" => 191.3, # 1 sec
    # "C107_025" => 191.3, # 3 sec
    # "C108_025" => 191.3, # 5 sec
    # "C109_025" => 191.3, # 13 sec

    # "RC101_025" => 461.1, # 30 sec with limit=10000   --> 4113 sec with limit 400 / 1e-7
    # # "RC102_025" => 351.8,  # oscillation? columns are added forever
    # "RC103_025" => 332.8, # 22 sec
    # "RC104_025" => 306.6, # 88 sec
    # "RC105_025" => 411.3, # 2 sec
    # "RC106_025" => 345.5, # 1 sec
    # "RC107_025" => 298.3, # 11 sec
    # "RC108_025" => 294.5, # 225 sec

    # # "RC101_050" => 944.0, # osciilation?
    # # "RC102_050" => 822.5, # osciilation?
    # # "RC103_050" => 710.9,
    # # "RC104_050" => 545.8,
    # # "RC105_050" => 855.3,
    # # "RC106_050" => 723.2,
    # # "RC107_050" => 642.7,
    # # "RC108_050" => 598.1,

    # "R101_050" => 1044.0, # 12 sec
    # "R102_050" => 909.0, # 10 sec
    # "R103_050" => 772.9, # complete B&B takes very long time
    # "R104_050" => 625.4,
    # "R105_050" => 899.3,
    # "R106_050" => 793.0,
    # "R107_050" => 711.1,
    # "R108_050" => ???,
    # "R109_050" => 786.8,
    # "R110_050" => 697.0,
    # "R111_050" => ???,
    # "R112_050" => ???,

)

comp_time = OrderedDict()

# (name, val) = pop!(solomon_dataset)
for (name, val) in solomon_dataset
    @info("Solving the Solomon Instance $name")
    solomon = load_solomon(name)
    vrptw = generate_solomon_vrptw_instance(solomon)

    @testset "$name" begin
        println("-- Solomon Instance $name --")
        start_time = time()
        @time routes, obj_val = solve_vrp_bnb(vrptw)
        comp_time[name] = time() - start_time
        @show obj_val
        @test isapprox(obj_val, val, atol=1e-7)
    end
end
