
to_int(x::String) = parse(Int, x)
to_float(x::String) = parse(Float64, x)



distance(n1::Node, n2::Node) = sqrt((n1.cx - n2.cx)^2 + (n1.cy - n2.cy)^2)

function calculate_solomon_cost(node::Array{Node})
    n_nodes = length(node)
    cost = zeros(n_nodes, n_nodes)
    for i in 1:n_nodes-1
        # cost[i, i] = Inf # This setup will break some solomon instances. Set to zero is necessary.
        for j in i+1:n_nodes
            c = distance(node[i], node[j])
            c = floor(10 * c) / 10 
            cost[i,j] = c
            cost[j,i] = c
        end
    end
    return cost
end


function read_solomon_data(data_filename)
    # data_filename = joinpath("solomon-1987-c1", "C101_100.xml")

    xdoc = parse_file(data_filename)
    xroot = root(xdoc)  # an instance of XMLElement

    xinfo = xroot["info"]
    dataset = xinfo[1]["dataset"][1] |> content
    data_name = xinfo[1]["name"][1] |> content

    # Nodes
    xnetwork = xroot["network"]
    xnodes = xnetwork[1]["nodes"][1]["node"]
    n_nodes = length(xnodes)
    node = Array{Node}(undef, n_nodes)
    for i in 1:n_nodes
        xnode = xnodes[i]
        id = attribute(xnode, "id") |> to_int
        type = attribute(xnode, "type") |> to_int
        cx = find_element(xnode, "cx") |> content |> to_float
        cy = find_element(xnode, "cy") |> content |> to_float

        if id == 0 
            node[end] = Node(id, type, cx, cy)
        else
            node[id] = Node(id, type, cx, cy)
        end
    end

    # Fleet 
    xfleet = xroot["fleet"][1]
    xvec = xfleet["vehicle_profile"][1]
    vehicle_type = attribute(xvec, "type") |> to_int
    n_vehicles = attribute(xvec, "number") |> to_int
    departure_node = find_element(xvec, "departure_node") |> content |> to_int
    arrival_node = find_element(xvec, "arrival_node") |> content |> to_int
    capacity = find_element(xvec, "capacity") |> content |> to_float
    max_travel_time = find_element(xvec, "max_travel_time") |> content |> to_float
    fleet = Fleet(vehicle_type, n_vehicles, 
                departure_node, arrival_node, capacity, max_travel_time)
    
    # Requests
    xrequests = xroot["requests"][1]["request"]
    n_requests = length(xrequests)
    request = Array{Request}(undef, n_requests)
    for i in 1:n_requests
        req = xrequests[i]
        id = attribute(req, "id") |> to_int
        node_id = attribute(req, "node") |> to_int
        tw = find_element(req, "tw")
        start_time = find_element(tw, "start") |> content |> to_int
        end_time = find_element(tw, "end") |> content |> to_int
        quantity = find_element(req, "quantity") |> content |> to_float
        service_time = find_element(req, "service_time") |> content |> to_float
        request[i] = Request(id, node_id, start_time, end_time, quantity, service_time)
    end

    return dataset, data_name, node, fleet, request
end

function load_solomon(dataset_name::String)
    data_file_path = dataset_name * ".xml"
    data_file_path = joinpath(@__DIR__, "..", "solomon-1987", data_file_path)
    dataset, data_name, nodes, fleet, requests = read_solomon_data(data_file_path)
    solomon = SolomonDataset(
        dataset_name,
        nodes,
        fleet,
        requests
    )
    return solomon
end

function generate_solomon_vrptw_instance(solomon::SolomonDataset)
    nodes = solomon.nodes 
    fleet = solomon.fleet
    requests = solomon.requests

    n_vehicles = fleet.number
    n_customers = length(requests)

    # Add a dummy node for depot at the end
    push!(nodes, nodes[end])
    depot0 = n_customers + 1
    depot_dummy = n_customers + 2
    cost = calculate_solomon_cost(nodes)
    cost[depot0, depot_dummy] = Inf
    cost[depot_dummy, depot0] = Inf

    n_nodes = length(nodes)
    @assert n_nodes - 2 == n_customers
    V = 1:n_vehicles
    N = 1:n_nodes
    C = 1:n_customers

    d = [requests[i].quantity for i in C]
    q = fleet.capacity
    a = [requests[i].start_time for i in C]
    b = [requests[i].end_time for i in C]
    t = copy(cost)
    θ = [requests[i].service_time for i in C]

    vrptw_inst = VRPTW_Instance(
        t,
        θ,
        a,
        b,
        d,
        q,
        fleet.max_travel_time
    )

    return vrptw_inst
end

function solomon_to_espprc(solomon::SolomonDataset, dual_var)
    nodes, fleet, requests = solomon.nodes, solomon.fleet, solomon.requests
    n_nodes = length(nodes)
    n_vehicles = fleet.number
    n_requests = length(requests)

    # Add a dummy node for depot at the end
    push!(nodes, nodes[n_nodes])
    depot0 = n_requests + 1
    depot_dummy = n_requests + 2

    n_nodes = length(nodes)
    V = 1:n_vehicles
    N = 1:n_nodes
    C = 1:n_requests

    d = [requests[i].quantity for i in C]
    q = fleet.capacity
    a = [requests[i].start_time for i in C]
    b = [requests[i].end_time for i in C]
    θ = [requests[i].service_time for i in C]

    t = calculate_solomon_cost(nodes)
    t[depot0, depot_dummy] = Inf
    t[depot_dummy, depot0] = Inf
    time_mtx = copy(t)

    origin = depot0
    destination = depot_dummy
    capacity = fleet.capacity
    dual_var = [dual_var; 0.0; 0.0]
    # alpha = rand(0:20, n_nodes) * 2
    # cost_mtx = t - repeat(alpha, 1, n_nodes)
    # cost_mtx = t .* (rand(size(t)...) .- 0.25)
    cost_mtx = copy(t)
    for i in N
        for j in N
            # println("($i, $j): dist=$(cost_mtx[i,j]), dv=$(dual_var[i])")
            cost_mtx[i,j] -= dual_var[i]
        end
    end

    resrc_mtx = zeros(n_nodes, n_nodes)
    for i in N, j in C
        resrc_mtx[i, j] = requests[j].quantity
    end
    early_time = [a; 0; 0]
    late_time = [b; 0; fleet.max_travel_time]
    service_time = [θ; 0; 0]
    travel_time = copy(t)


    ei = ESPPRC_Instance(
        origin,
        destination,
        capacity,
        cost_mtx,
        travel_time,
        resrc_mtx,
        early_time,
        late_time,
        service_time
    )
end
