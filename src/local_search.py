import random

from src.drone_scheduler import create_drone_schedule
from src.models import Solution
from src.evaluator import evaluate_solution


def swap_two_truck_nodes(truck_route):
    if len(truck_route) <= 4:
        return truck_route[:]

    new_route = truck_route[:]
    i, j = random.sample(range(1, len(new_route) - 1), 2)
    new_route[i], new_route[j] = new_route[j], new_route[i]

    return new_route


def reverse_truck_segment(truck_route):
    if len(truck_route) <= 4:
        return truck_route[:]

    new_route = truck_route[:]
    i, j = sorted(random.sample(range(1, len(new_route) - 1), 2))
    new_route[i:j + 1] = reversed(new_route[i:j + 1])

    return new_route


def insert_truck_node(truck_route):
    if len(truck_route) <= 4:
        return truck_route[:]

    new_route = truck_route[:]
    i, j = random.sample(range(1, len(new_route) - 1), 2)
    node = new_route.pop(i)
    new_route.insert(j, node)

    return new_route


def generate_neighbor_route(truck_route):
    operator = random.choice([
        swap_two_truck_nodes,
        reverse_truck_segment,
        insert_truck_node
    ])

    return operator(truck_route)


def or_opt_drone_subroutes(drone_assignments, drone_time, battery_capacity):
    """
    Or-Opt local search for drone sub-routes.
    It relocates segments of length 1, 2, or 3 inside each drone path.
    """

    improved_assignments = {}

    for edge, path in drone_assignments.items():
        start_node, end_node = edge
        best_path = path[:]

        def route_cost(route):
            if len(route) == 0:
                return drone_time[start_node][end_node]

            total = drone_time[start_node][route[0]]

            for i in range(len(route) - 1):
                total += drone_time[route[i]][route[i + 1]]

            total += drone_time[route[-1]][end_node]
            return total

        best_cost = route_cost(best_path)
        improved = True

        while improved:
            improved = False

            for segment_length in [1, 2, 3]:
                if len(best_path) < segment_length:
                    continue

                for i in range(len(best_path) - segment_length + 1):
                    segment = best_path[i:i + segment_length]
                    remaining = best_path[:i] + best_path[i + segment_length:]

                    for j in range(len(remaining) + 1):
                        candidate = remaining[:j] + segment + remaining[j:]
                        candidate_cost = route_cost(candidate)

                        if candidate_cost < best_cost and candidate_cost <= battery_capacity:
                            best_path = candidate
                            best_cost = candidate_cost
                            improved = True
                            break

                    if improved:
                        break

                if improved:
                    break

        improved_assignments[edge] = best_path

    return improved_assignments


def improve_solution_once(
    current_solution,
    all_nodes,
    truck_time,
    drone_time,
    battery_capacity
):
    candidate_truck_route = generate_neighbor_route(
        current_solution.truck_route
    )

    candidate_truck_route, candidate_drone_assignments = create_drone_schedule(
        candidate_truck_route,
        all_nodes,
        drone_time,
        battery_capacity
    )

    candidate_drone_assignments = or_opt_drone_subroutes(
        candidate_drone_assignments,
        drone_time,
        battery_capacity
    )

    candidate_solution = Solution(
        truck_route=candidate_truck_route,
        drone_assignments=candidate_drone_assignments
    )

    candidate_solution = evaluate_solution(
        candidate_solution,
        truck_time,
        drone_time,
        battery_capacity
    )

    return candidate_solution