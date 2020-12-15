using LightXML

to_int(x::String) = parse(Int, x)
to_float(x::String) = parse(Float64, x)

struct Node
    id::Int
    type::Int
    cx::Float64
    cy::Float64
end

distance(n1::Node, n2::Node) = sqrt((n1.cx - n2.cx)^2 + (n1.cy - n2.cy)^2)

function calculate_cost(node::Array{Node})
    n_nodes = length(node)
    cost = zeros(n_nodes, n_nodes)
    for i in 1:n_nodes-1
        for j in i+1:n_nodes
            c = distance(node[i], node[j])
            cost[i,j] = c
            cost[j,i] = c
        end
    end
    return cost
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

function readdata(data_filename)
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