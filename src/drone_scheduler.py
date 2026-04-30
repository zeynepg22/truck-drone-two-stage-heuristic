def get_truck_edges(truck_route):
    """
    Converts truck route into consecutive edges.

    Example:
    [0, 3, 5, 10] -> [(0, 3), (3, 5), (5, 10)]
    """
    return [
        (truck_route[i], truck_route[i + 1])
        for i in range(len(truck_route) - 1)
    ]


def drone_subroute_cost(start_node, drone_nodes, end_node, drone_time):
    """
    Calculates drone travel time for:
    start_node -> drone_nodes -> end_node
    """
    total = 0
    current_node = start_node

    for node in drone_nodes:
        total += drone_time[current_node][node]
        current_node = node

    total += drone_time[current_node][end_node]

    return total


def assign_drone_customers_to_edges(truck_route, all_nodes, drone_time):
    """
    Assigns customers not visited by the truck to the best truck edge.

    The selected edge is the one that creates the smallest extra drone travel time.
    """
    truck_nodes = set(truck_route)
    drone_only_nodes = [
        node for node in all_nodes
        if node not in truck_nodes
    ]

    truck_edges = get_truck_edges(truck_route)
    drone_assignments = {edge: [] for edge in truck_edges}

    for customer in drone_only_nodes:
        best_edge = None
        best_extra_cost = float("inf")

        for edge in truck_edges:
            start_node, end_node = edge

            extra_cost = (
                drone_time[start_node][customer]
                + drone_time[customer][end_node]
                - drone_time[start_node][end_node]
            )

            if extra_cost < best_extra_cost:
                best_extra_cost = extra_cost
                best_edge = edge

        drone_assignments[best_edge].append(customer)

    return drone_assignments


def order_drone_nodes_nearest_neighbor(start_node, drone_nodes, drone_time):
    """
    Orders drone-only nodes using nearest neighbor.
    """
    unvisited = drone_nodes[:]
    ordered_nodes = []
    current_node = start_node

    while unvisited:
        nearest_node = min(
            unvisited,
            key=lambda node: drone_time[current_node][node]
        )

        ordered_nodes.append(nearest_node)
        unvisited.remove(nearest_node)
        current_node = nearest_node

    return ordered_nodes


def order_all_drone_assignments(drone_assignments, drone_time):
    """
    Orders all drone sub-routes.
    """
    ordered_assignments = {}

    for edge, drone_nodes in drone_assignments.items():
        start_node, _ = edge

        ordered_nodes = order_drone_nodes_nearest_neighbor(
            start_node,
            drone_nodes,
            drone_time
        )

        ordered_assignments[edge] = ordered_nodes

    return ordered_assignments


def rebuild_drone_assignments(truck_route, remaining_drone_nodes, drone_time):
    """
    Rebuilds drone assignments after truck route changes.
    """
    truck_edges = get_truck_edges(truck_route)
    drone_assignments = {edge: [] for edge in truck_edges}

    for customer in remaining_drone_nodes:
        best_edge = None
        best_extra_cost = float("inf")

        for edge in truck_edges:
            start_node, end_node = edge

            extra_cost = (
                drone_time[start_node][customer]
                + drone_time[customer][end_node]
                - drone_time[start_node][end_node]
            )

            if extra_cost < best_extra_cost:
                best_extra_cost = extra_cost
                best_edge = edge

        drone_assignments[best_edge].append(customer)

    return order_all_drone_assignments(drone_assignments, drone_time)


def repair_battery_violations(truck_route, drone_assignments, drone_time, battery_capacity):
    """
    Repairs drone sub-routes that exceed battery capacity.

    If a drone sub-route is infeasible, one drone customer is moved back to the truck route.
    """
    changed = True

    while changed:
        changed = False

        for edge in list(drone_assignments.keys()):
            start_node, end_node = edge
            drone_nodes = drone_assignments[edge]

            cost = drone_subroute_cost(
                start_node,
                drone_nodes,
                end_node,
                drone_time
            )

            if cost > battery_capacity and drone_nodes:
                moved_node = drone_nodes.pop()

                insert_index = truck_route.index(end_node)
                truck_route.insert(insert_index, moved_node)

                remaining_drone_nodes = []
                for nodes in drone_assignments.values():
                    remaining_drone_nodes.extend(nodes)

                drone_assignments = rebuild_drone_assignments(
                    truck_route,
                    remaining_drone_nodes,
                    drone_time
                )

                changed = True
                break

    return truck_route, drone_assignments


def create_drone_schedule(truck_route, all_nodes, drone_time, battery_capacity):
    """
    Full Stage 2 function.

    Step 1: Assign drone-only customers to truck edges.
    Step 2: Order drone customers inside each edge.
    Step 3: Repair battery violations.
    """
    drone_assignments = assign_drone_customers_to_edges(
        truck_route,
        all_nodes,
        drone_time
    )

    drone_assignments = order_all_drone_assignments(
        drone_assignments,
        drone_time
    )

    truck_route, drone_assignments = repair_battery_violations(
        truck_route,
        drone_assignments,
        drone_time,
        battery_capacity
    )

    return truck_route, drone_assignments