def route_distance(route, distance_matrix):
    """
    Calculates total distance/time of a given route.
    """
    total = 0

    for i in range(len(route) - 1):
        current_node = route[i]
        next_node = route[i + 1]
        total += distance_matrix[current_node][next_node]

    return total


def nearest_neighbor_truck_route(distance_matrix, start_node, end_node):
    """
    Creates an initial truck route using nearest neighbor heuristic.

    The truck starts from origin, visits customer nodes, and finishes at destination.
    """
    n = len(distance_matrix)

    unvisited = set(range(n))
    unvisited.remove(start_node)
    unvisited.remove(end_node)

    route = [start_node]
    current_node = start_node

    while unvisited:
        nearest_node = min(
            unvisited,
            key=lambda node: distance_matrix[current_node][node]
        )

        route.append(nearest_node)
        unvisited.remove(nearest_node)
        current_node = nearest_node

    route.append(end_node)

    return route


def two_opt(route, distance_matrix):
    """
    Improves the truck route using 2-opt.

    2-opt reverses route segments if doing so reduces the total route cost.
    """
    best_route = route[:]
    improved = True

    while improved:
        improved = False

        for i in range(1, len(best_route) - 2):
            for j in range(i + 1, len(best_route) - 1):
                candidate_route = (
                    best_route[:i]
                    + best_route[i:j + 1][::-1]
                    + best_route[j + 1:]
                )

                if route_distance(candidate_route, distance_matrix) < route_distance(best_route, distance_matrix):
                    best_route = candidate_route
                    improved = True

    return best_route


def reduce_truck_route_for_drone_usage(truck_route, keep_every=2):
    """
    Reduces truck route so that some customers can be assigned to the drone.

    If the truck visits every customer, the drone has no role.
    This function keeps only some customers as truck/synchronization nodes.
    """
    if len(truck_route) <= 3:
        return truck_route

    reduced_route = [truck_route[0]]
    middle_nodes = truck_route[1:-1]

    for index, node in enumerate(middle_nodes):
        if index % keep_every == 0:
            reduced_route.append(node)

    reduced_route.append(truck_route[-1])

    return reduced_route


def create_initial_truck_route(distance_matrix, start_node, end_node, keep_every=2):
    """
    Full Stage 1 function.

    Step 1: Create full truck route with nearest neighbor.
    Step 2: Improve it using 2-opt.
    Step 3: Reduce truck route to allow drone usage.
    """
    initial_route = nearest_neighbor_truck_route(
        distance_matrix,
        start_node,
        end_node
    )

    improved_route = two_opt(initial_route, distance_matrix)

    reduced_route = reduce_truck_route_for_drone_usage(
        improved_route,
        keep_every=keep_every
    )

    return reduced_route, improved_route