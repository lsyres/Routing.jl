using VRPTW
using Test
using DataStructures

solomon_dataset = OrderedDict(
    "R101_025" => 617.1,
    "R102_025" => 547.1,
    "R103_025" => 454.6,
    "R104_025" => 416.9, # 3 sec
    "R105_025" => 530.5, # 1 sec
    "R106_025" => 465.4, # 80 sec
    "R107_025" => 424.3, # 2 sec
    "R108_025" => 397.3, # 5 sec
    "R109_025" => 441.3, # 1 sec
    "R110_025" => 444.1, # 40 sec
    "R111_025" => 428.8, # 21 sec
    "R112_025" => 393.0, # 21 sec

    "C101_025" => 191.3, # 1 sec
    "C102_025" => 190.3, # 29 sec
    "C103_025" => 190.3, # 77 sec with limit=10000
    "C104_025" => 186.9, # 250 sec with limit=10000
    "C105_025" => 191.3, # 2 sec
    "C106_025" => 191.3, # 1 sec
    "C107_025" => 191.3, # 3 sec
    "C108_025" => 191.3, # 4 sec
    "C109_025" => 191.3, # 30 sec with limit=10000    

    "RC101_025" => 461.1, # 30 sec with limit=10000    
    # "RC102_025" => 351.8,  # oscillation? columns are added forever
    "RC103_025" => 332.9, # 22 sec
    "RC104_025" => 306.6, # 117 sec
    "RC105_025" => 411.3, # 112 sec
    "RC106_025" => 345.5, # 1 sec
    "RC107_025" => 298.3, # 8 sec with limit=10000
    "RC108_025" => 294.5, # 170 sec with limit=10000 / 59 sec with limit=400

    # "RC101_050" => 944.0, # osciilation?
    # "RC102_050" => 822.5, # osciilation?
    # "RC103_050" => 710.9,
    # "RC104_050" => 545.8,
    # "RC105_050" => 855.3,
    # "RC106_050" => 723.2,
    # "RC107_050" => 642.7,
    # "RC108_050" => 598.1,

    # "R101_050" => 1044.0, # 37 sec
    # "R102_050" => 909.0, # 11 sec
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

(name, val) = pop!(solomon_dataset)
# for (name, val) in solomon_dataset
    @info("Solving the Solomon Instance $name")
    solomon = load_solomon(name)
    vrptw = generate_solomon_vrptw_instance(solomon)

    @testset "$name" begin
        println("-- Solomon Instance $name --")
        @time routes, obj_val = solve_vrp_bnb(vrptw)
        @show obj_val
        @test isapprox(obj_val, val, atol=1e-7)
    end
# end
